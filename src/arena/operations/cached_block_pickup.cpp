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

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/arena/operations/block_extent_set.hpp"
#include "cosm/arena/operations/cache_extent_clear.hpp"
#include "cosm/arena/operations/free_block_drop.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/ds/operations/cell2D_empty.hpp"
#include "cosm/fsm/cell2D_fsm.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/repr/operations/block_pickup.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, operations, detail);
using carepr::base_cache;
using cds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cached_block_pickup::cached_block_pickup(carepr::arena_cache* cache,
                                         crepr::base_block3D* pickup_block,
                                         cpal::argos_sm_adaptor* sm,
                                         const rtypes::type_uuid& robot_id,
                                         const rtypes::timestep& t)
    : ER_CLIENT_INIT("cosm.arena.operations.cached_block_pickup"),
      cell2D_op(cache->dcenter2D()),
      mc_robot_id(robot_id),
      mc_timestep(t),
      m_real_cache(cache),
      m_sm(sm),
      m_pickup_block(pickup_block) {
  ER_ASSERT(m_real_cache->n_blocks() >= base_cache::kMinBlocks,
            "< %zu blocks in cache",
            base_cache::kMinBlocks);
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cached_block_pickup::visit(cfsm::cell2D_fsm& fsm) {
  fsm.event_block_pickup();
} /* visit() */

void cached_block_pickup::visit(cds::cell2D& cell) {
  ER_ASSERT(cell.state_has_cache(),
            "Cell@%s not in HAS_CACHE [state=%d]",
            rcppsw::to_string(cell.loc()).c_str(),
            cell.fsm().current_state());
  visit(cell.fsm());
  ER_ASSERT(cell.state_has_cache(),
            "Cell@%s not in HAS_CACHE [state=%d]",
            rcppsw::to_string(cell.loc()).c_str(),
            cell.fsm().current_state());
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

  auto cache_coord = m_real_cache->dcenter2D();
  ER_ASSERT(cache_coord == cell2D_op::coord(),
            "Coordinates for cache%d%s/host cell@%s do not agree",
            cache_id.v(),
            cache_coord.to_str().c_str(),
            rcppsw::to_string(coord()).c_str());

  cds::cell2D& cell = map.access<arena_grid::kCell>(coord());
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

    ER_INFO("Block%d picked up from cache%d@%s,remaining=[%s] (%zu)",
            m_pickup_block->id().v(),
            cache_id.v(),
            rcppsw::to_string(coord()).c_str(),
            rcppsw::to_string(m_real_cache->blocks()).c_str(),
            m_real_cache->n_blocks());
  } else {
    /* Already holding cache mutex */
    visit(*m_real_cache);
    ER_ASSERT(1 == m_real_cache->blocks().size(),
              "Cache%d@%s incorrect # blocks: %zu != 1",
              m_real_cache->id().v(),
              rcppsw::to_string(m_real_cache->dcenter2D()).c_str(),
              m_real_cache->blocks().size());
    m_orphan_block = m_real_cache->block_select(nullptr);

    map.lock_wr(map.grid_mtx());

    /* Clear the cache extent cells (already holding cache mutex) */
    cache_extent_clear_visitor clear_op1(m_real_cache);
    clear_op1.visit(map.decoratee());

    /* clear cache host cell */
    cdops::cell2D_empty_visitor clear_op2(coord());
    clear_op2.visit(cell);

    /* "Drop" orphan block on the old cache host cell */
    caops::free_block_drop_visitor drop_op(
        m_orphan_block,
        coord(),
        map.grid_resolution(),
        arena_map_locking::ekCACHES_AND_GRID_HELD);
    drop_op.visit(cell);
    drop_op.visit(*m_orphan_block);

    /* set block extent cells for the orphan block */
    block_extent_set_visitor set_op(m_orphan_block);
    set_op.visit(map.decoratee());

    map.unlock_wr(map.grid_mtx());

    ER_ASSERT(cell.state_has_block(),
              "Depleted cache host cell@%s not in HAS_BLOCK",
              rcppsw::to_string(coord()).c_str());
    ER_ASSERT(cell.block3D() == m_orphan_block,
              "Cell@%s does not refer to orphan block%d",
              rcppsw::to_string(coord()).c_str(),
              m_orphan_block->id().v());

    /* remove the cache from the arena (already holding cache mutex) */
    map.cache_remove(m_real_cache, m_sm);

    ER_INFO("Block%d picked up from cache%d@%s [depleted]",
            m_pickup_block->id().v(),
            cache_id.v(),
            rcppsw::to_string(coord()).c_str());
    ER_INFO("Orphan block%d@%s released to arena",
            m_orphan_block->id().v(),
            rcppsw::to_string(m_orphan_block->danchor2D()).c_str());
  }
  /* finally, update state of arena map owned pickup block */
  visit(*m_pickup_block, &map);
} /* visit() */

void cached_block_pickup::visit(crepr::base_block3D& block,
                                caching_arena_map* map) {
  crops::block_pickup op(mc_robot_id, mc_timestep);

  /* need to take mutex--not held in caller */
  map->lock_wr(map->block_mtx());
  op.visit(block, crops::block_pickup_owner::ekARENA_MAP);
  map->unlock_wr(map->block_mtx());
} /* visit() */

NS_END(detail, operations, arena, cosm);
