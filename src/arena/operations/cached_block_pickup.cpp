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

#include "rcppsw/utils/maskable_enum.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/arena/operations/block_extent_set.hpp"
#include "cosm/arena/operations/cache_extent_clear.hpp"
#include "cosm/arena/operations/free_block_drop.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/ds/operations/cell2D_empty.hpp"
#include "cosm/fsm/cell2D_fsm.hpp"
#include "cosm/repr/operations/block_pickup.hpp"
#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, operations, detail);
using cads::arena_grid;
using carepr::base_cache;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cached_block_pickup::cached_block_pickup(carepr::arena_cache* cache,
                                         crepr::sim_block3D* pickup_block,
                                         cpargos::swarm_manager_adaptor* sm,
                                         const rtypes::type_uuid& robot_id,
                                         const rtypes::timestep& t,
                                         const locking& locking)
    : ER_CLIENT_INIT("cosm.arena.operations.cached_block_pickup"),
      cell2D_op(cache->dcenter2D()),
      mc_robot_id(robot_id),
      mc_timestep(t),
      mc_locking(locking),
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

  map.maybe_lock_wr(map.cache_mtx(), !(mc_locking & locking::ekCACHES_HELD));
  /*
   * If there are more than kMinBlocks blocks in cache, just remove one, and
   * update the underlying cell. If there are only kMinBlocks left, do the same
   * thing but also remove the cache, as a cache with less than that many blocks
   * is not a cache.
   */
  if (m_real_cache->n_blocks() > base_cache::kMinBlocks) {
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
            rcppsw::to_string(m_real_cache->n_blocks()).c_str(),
            m_real_cache->n_blocks());
  } else {
    visit(*m_real_cache);
    ER_ASSERT(1 == m_real_cache->n_blocks(),
              "Cache%d@%s incorrect # blocks: %zu != 1",
              m_real_cache->id().v(),
              rcppsw::to_string(m_real_cache->dcenter2D()).c_str(),
              m_real_cache->n_blocks());
    auto orphan_block = m_real_cache->block_select(nullptr);

    /*
     * We need the cache+grid mutex around clearing out the old cache
     * celnl/extent, and the also the block mutex for dropping the orphan
     * block. We acquire them all here, even though its a bit early, because we
     * DO always need the locks to be acquired in the same order to avoid
     * deadlocks. See COSM#151.
     */
    map.maybe_lock_wr(map.grid_mtx(), !(mc_locking & locking::ekGRID_HELD));

    /* Clear the cache extent cells (already holding cache mutex) */
    cache_extent_clear_visitor clear_op1(m_real_cache);
    clear_op1.visit(map.decoratee());

    /* clear cache host cell */
    cdops::cell2D_empty_visitor clear_op2(coord());
    clear_op2.visit(cell);

    /*
     * Remove the cache from the arena. This must be BEFORE dropping the orphan
     * block on the old host cell, so that the drop is an actual drop and does
     * not trigger block distribution due to a cache conflict.
     */
    map.cache_remove(m_real_cache, m_sm);

    /* "Drop" orphan block on the old cache host cell. */
    caops::free_block_drop_visitor drop_op(
        orphan_block, coord(), map.grid_resolution(), locking::ekALL_HELD);
    drop_op.visit(map);
    ER_INFO("Orphan block%d@%s released to arena",
            orphan_block->id().v(),
            rcppsw::to_string(orphan_block->danchor2D()).c_str());

    map.maybe_unlock_wr(map.grid_mtx(), !(mc_locking & locking::ekGRID_HELD));

    ER_ASSERT(cell.state_has_block(),
              "Depleted cache host cell@%s not in HAS_BLOCK",
              rcppsw::to_string(coord()).c_str());
    ER_ASSERT(cell.block3D() == orphan_block,
              "Cell@%s does not refer to orphan block%d",
              rcppsw::to_string(coord()).c_str(),
              orphan_block->id().v());

    ER_INFO("Block%d picked up from cache%d@%s [depleted]",
            m_pickup_block->id().v(),
            cache_id.v(),
            rcppsw::to_string(coord()).c_str());
  }
  map.maybe_lock_wr(map.block_mtx(), !(mc_locking & locking::ekBLOCKS_HELD));

  /* finally, update state of arena map owned pickup block */
  visit(*m_pickup_block);

  map.maybe_unlock_wr(map.block_mtx(), !(mc_locking & locking::ekBLOCKS_HELD));

  map.maybe_unlock_wr(map.cache_mtx(), !(mc_locking & locking::ekCACHES_HELD));
} /* visit() */

void cached_block_pickup::visit(crepr::sim_block3D& block) {
  crops::block_pickup op(mc_robot_id, mc_timestep);
  op.visit(block, crops::block_pickup_owner::ekARENA_MAP);
} /* visit() */

NS_END(detail, operations, arena, cosm);
