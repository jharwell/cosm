/**
 * \file cached_block_pickup.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/arena/operations/cached_block_pickup.hpp"

#include "cosm/ds/operations/cell2D_empty.hpp"
#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/fsm/cell2D_fsm.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/arena/operations/cache_extent_clear.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, operations, detail);
using cds::arena_grid;
using carepr::base_cache;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cached_block_pickup::cached_block_pickup(
    carepr::arena_cache* cache,
    cpal::argos_sm_adaptor* sm,
    const rtypes::type_uuid& robot_id,
    const rtypes::timestep& t)
    : ER_CLIENT_INIT("cosm.operations.cached_block_pickup"),
      cell2D_op(cache->dpos2D()),
      mc_robot_id(robot_id),
      mc_timestep(t),
      m_real_cache(cache),
      m_sm(sm) {
  ER_ASSERT(m_real_cache->n_blocks() >= base_cache::kMinBlocks,
            "< %zu blocks in cache",
            base_cache::kMinBlocks);
  m_pickup_block = m_real_cache->oldest_block();
  ER_ASSERT(nullptr != m_pickup_block, "No block in non-empty cache");
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cached_block_pickup::visit(cfsm::cell2D_fsm& fsm) {
  fsm.event_block_pickup();
} /* visit() */

void cached_block_pickup::visit(cds::cell2D& cell) {
  ER_ASSERT(0 != cell.loc().x() && 0 != cell.loc().y(),
            "Cell does not have coordinates");
  ER_ASSERT(cell.state_has_cache(), "Cell does not have cache");
  if (nullptr != m_orphan_block) {
    cell.entity(m_orphan_block);
    ER_DEBUG("Cell@%s gets orphan block%d after cache depletion",
             cell.loc().to_str().c_str(),
             m_orphan_block->id().v());
  }
  visit(cell.fsm());
} /* visit() */

void cached_block_pickup::visit(carepr::arena_cache& cache) {
  cache.block_remove(m_pickup_block);
  cache.has_block_pickup();
} /* visit() */

void cached_block_pickup::visit(caching_arena_map& map) {
  ER_ASSERT(m_real_cache->n_blocks() >= base_cache::kMinBlocks,
            "< %zu blocks in cache",
            base_cache::kMinBlocks);
  rtypes::type_uuid cache_id = m_real_cache->id();
  ER_ASSERT(rtypes::constants::kNoUUID != cache_id,
            "Cache ID undefined on block pickup");

  rmath::vector2z cache_coord = m_real_cache->dpos2D();
  ER_ASSERT(cache_coord == cell2D_op::coord(),
            "Coordinates for cache%d%s/cell@%s do not agree",
            cache_id.v(),
            cache_coord.to_str().c_str(),
            cell2D_op::coord().to_str().c_str());

  cds::cell2D& cell =
      map.access<arena_grid::kCell>(cell2D_op::x(), cell2D_op::y());
  ER_ASSERT(m_real_cache->n_blocks() == cell.block_count(),
            "Cache%d/cell@%s disagree on # of blocks: cache=%zu/cell=%zu",
            m_real_cache->id().v(),
            cell.loc().to_str().c_str(),
            m_real_cache->n_blocks(),
            cell.block_count());
  /*
   * If there are more than kMinBlocks blocks in cache, just remove one, and
   * update the underlying cell. If there are only kMinBlocks left, do the same
   * thing but also remove the cache, as a cache with less than that many blocks
   * is not a cache.
   */
  if (m_real_cache->n_blocks() > base_cache::kMinBlocks) {
    /* Already holding cache mutex */
    visit(*m_real_cache);

    /*
     * Do not need to hold grid mutex because we know we are the only robot
     * picking up from the cache right now (though others can do it later) this
     * timestep, and caches by definition have a unique location, AND that it is
     * not possible for another robot's \ref nest_block_drop to trigger a block
     * re-distribution to the cache host cell right now (re-distribution avoids
     * caches).
     */
    visit(cell);

    ER_ASSERT(cell.state_has_cache(),
              "Cell@(%u, %u) with >= %zu blocks does not have cache",
              cell2D_op::x(),
              cell2D_op::y(),
              base_cache::kMinBlocks);

    ER_INFO("fb%u: block%d from cache%d@(%u, %u),remaining=[%s] (%zu)",
            mc_robot_id.v(),
            m_pickup_block->id().v(),
            cache_id.v(),
            cell2D_op::x(),
            cell2D_op::y(),
            rcppsw::to_string(m_real_cache->blocks()).c_str(),
            m_real_cache->n_blocks());
  } else {
    /* Already holding cache mutex */
    visit(*m_real_cache);
    m_orphan_block = m_real_cache->oldest_block();
    /*
     * Do not need to hold grid mutex because all caches have a unique location,
     * and when this one is depleted there will still be a block on the host
     * cell, so block distribution will avoid it regardless.
     */
    visit(cell);

    ER_ASSERT(cell.state_has_block(),
              "cell@(%u, %u) with 1 block has cache",
              cell2D_op::x(),
              cell2D_op::y());

    /* Already holding cache mutex */
    cache_extent_clear op(coord(), m_real_cache);
    op.visit(map);

    /* Already holding cache mutex */
    map.cache_remove(m_real_cache, m_sm);

    ER_INFO("fb%u: block%d from cache%d@(%u, %u) [depleted]",
            mc_robot_id.v(),
            m_pickup_block->id().v(),
            cache_id.v(),
            cell2D_op::x(),
            cell2D_op::y());
  }
  std::scoped_lock lock(*map.block_mtx());
  visit(*m_pickup_block);
} /* visit() */

void cached_block_pickup::visit(crepr::base_block3D& block) {
  ER_ASSERT(rtypes::constants::kNoUUID != block.id(), "Unamed block");
  block.robot_pickup_event(mc_robot_id, mc_timestep);

  ER_INFO("Block%d is now carried by fb%u", block.id().v(), mc_robot_id.v());
} /* visit() */

NS_END(detail, operations, arena, cosm);
