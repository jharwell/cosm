/**
 * \file cache_block_drop.cpp
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
#include "cosm/arena/operations/cache_block_drop.hpp"

#include "rcppsw/utils/maskable_enum.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/arena/operations/free_block_drop.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, operations, detail);
using cds::arena_grid;

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_block_drop::cache_block_drop(crepr::base_block3D* arena_block,
                                   carepr::arena_cache* cache,
                                   const rtypes::discretize_ratio& resolution,
                                   const locking& locking)
    : ER_CLIENT_INIT("cosm.arena.operations.cache_block_drop"),
      cell2D_op(cache->dcenter2D()),
      mc_locking(locking),
      mc_resolution(resolution),
      m_arena_block(arena_block),
      m_cache(cache) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_block_drop::visit(cds::cell2D& cell) {
  ER_ASSERT(0 != cell.loc().x() && 0 != cell.loc().y(),
            "Cell does not have coordinates");

  visit(cell.fsm());
  ER_ASSERT(m_cache->n_blocks() == cell.block_count(),
            "Cache/cell disagree on # of blocks: cache=%zu/cell=%zu",
            m_cache->n_blocks(),
            cell.block_count());
} /* visit() */

void cache_block_drop::visit(fsm::cell2D_fsm& fsm) {
  ER_ASSERT(fsm.state_has_cache(), "Cell does not contain a cache");
  fsm.event_block_drop();
} /* visit() */

void cache_block_drop::visit(caching_arena_map& map) {
  /*
   * We might be modifying a cell--don't want block distribution in ANOTHER
   * thread to pick our chosen cell for distribution.
   */
  map.maybe_lock_wr(map.block_mtx(),
                    !(mc_locking & locking::ekBLOCKS_HELD));

  visit(*m_arena_block);
  map.maybe_unlock_wr(map.block_mtx(),
                      !(mc_locking & locking::ekBLOCKS_HELD));

  map.maybe_lock_wr(map.cache_mtx(),
                    !(mc_locking & locking::ekCACHES_HELD));
  visit(*m_cache);
  map.maybe_unlock_wr(map.cache_mtx(),
                      !(mc_locking & locking::ekCACHES_HELD));

  /*
   * Do not need to hold grid mutex (but might be) because we know we are the
   * only robot picking up from the cache right now (though others can do it
   * later) this timestep, and caches by definition have a unique location, AND
   * if another robot has just caused a block re-distribution, that operation
   * avoids caches.
   */
  visit(map.access<arena_grid::kCell>(cell2D_op::coord()));

  ER_INFO("Block%d dropped in cache%d,total=[%s] (%zu)",
          m_arena_block->id().v(),
          m_cache->id().v(),
          rcppsw::to_string(m_cache->blocks()).c_str(),
          m_cache->n_blocks());
} /* visit() */

void cache_block_drop::visit(crepr::base_block3D& block) {
  auto visitor = operations::free_block_drop_visitor::for_block(
      rmath::vector2z(cell2D_op::x(), cell2D_op::y()), mc_resolution);
  visitor.visit(block);
} /* visit() */

void cache_block_drop::visit(carepr::arena_cache& cache) {
  cache.block_add(m_arena_block);
  cache.has_block_drop();
} /* visit() */

NS_END(detail, operations, arena, cosm);
