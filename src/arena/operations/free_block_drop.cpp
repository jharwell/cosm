/**
 * \file free_block_drop.cpp
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
#include "cosm/arena/operations/free_block_drop.hpp"

#include "rcppsw/utils/maskable_enum.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/arena/operations/cache_block_drop.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/repr/base_block3D.hpp"
#include "cosm/ds/operations/cell2D_block_extent.hpp"
#include "cosm/arena/operations/block_extent_set.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, operations, detail);
using cds::arena_grid;

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
free_block_drop free_block_drop::for_block(
    const rmath::vector2z& coord,
    const rtypes::discretize_ratio& resolution) {
  return free_block_drop({}, /* empty variant */
                         coord,
                         resolution,
                         arena_map_locking::ekNONE_HELD);
} /* for_block() */

static bool block_drop_overlap_with_cache(const crepr::base_block3D* block,
                                          const carepr::arena_cache* cache,
                                          const rmath::vector2d& drop_loc);

static bool block_drop_overlap_with_nest(const crepr::base_block3D* block,
                                         const crepr::nest& nest,
                                         const rmath::vector2d& drop_loc);

static bool block_drop_loc_conflict(const base_arena_map& map,
                                    const crepr::base_block3D* block,
                                    const rmath::vector2d& loc);

static bool block_drop_loc_conflict(const caching_arena_map& map,
                                    const crepr::base_block3D* block,
                                    const rmath::vector2d& loc);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
free_block_drop::free_block_drop(crepr::base_block3D* block,
                                 const rmath::vector2z& coord,
                                 const rtypes::discretize_ratio& resolution,
                                 const arena_map_locking& locking)
    : ER_CLIENT_INIT("cosm.arena.operations.free_block_drop"),
      cell2D_op(coord),
      mc_resolution(resolution),
      mc_locking(locking),
      m_block(block) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void free_block_drop::visit(cds::cell2D& cell) {
  visit(*m_block);
  visit(cell.fsm());

  /*
   * We can't set the entity or color undconditionally, because this event is
   * also triggered on block drops into caches.
   */
  if (cell.state_has_block()) {
    cell.entity(m_block);
    cell.color(m_block->md()->color());
  }
} /* visit() */

void free_block_drop::visit(fsm::cell2D_fsm& fsm) {
  fsm.event_block_drop();
} /* visit() */

void free_block_drop::visit(crepr::base_block3D& block) {
  block.md()->robot_id_reset();

  auto rloc = rmath::vector3d(rmath::zvec2dvec(cell2D_op::coord(),
                                               mc_resolution.v()));

  block.ranchor3D(rloc);
  block.danchor3D(rmath::vector3z(cell2D_op::coord()));
} /* visit() */

void free_block_drop::visit(base_arena_map& map) {
  map.maybe_lock(map.block_mtx(),
                 !(mc_locking & arena_map_locking::ekBLOCKS_HELD));

  /*
   * We might be modifying this cell--don't want block distribution in ANOTHER
   * thread to pick this cell for distribution.
   */
  map.maybe_lock(map.grid_mtx(), !(mc_locking & arena_map_locking::ekGRID_HELD));

  auto rloc = rmath::zvec2dvec(cell2D_op::coord(), mc_resolution.v());
  bool conflict = block_drop_loc_conflict(map, m_block, rloc);

  cds::cell2D& cell = map.access<arena_grid::kCell>(cell2D_op::coord());

  /*
   * Dropping a block onto a cell that already contains a single block (but not
   * a cache) does not work. Failing to do this results robots that are carrying
   * a block and that abort their current task causing the cell that they drop
   * the block onto to go into a HAS_CACHE state, when the cell entity is not a
   * cache.
   */
  if (cell.state_has_block() || conflict) {
    map.distribute_single_block(m_block, arena_map_locking::ekALL_HELD);
  } else {
    /*
     * Cell does not have a block/cache on it, so it is safe to drop the block
     * on it and change the cell state.
     *
     * Holding arena map grid lock, block lock if locking enabled.
     */
    visit(cell);

    /* set block extent */
    caops::block_extent_set_visitor e(m_block);
    e.visit(map.decoratee());
  }

  map.maybe_unlock(map.grid_mtx(),
                   !(mc_locking & arena_map_locking::ekGRID_HELD));
  map.maybe_unlock(map.block_mtx(),
                   !(mc_locking & arena_map_locking::ekBLOCKS_HELD));
} /* visit() */

void free_block_drop::visit(caching_arena_map& map) {
  /* needed for atomic check for cache overlap+do drop operation */
  map.maybe_lock(map.cache_mtx(),
                 !(mc_locking & arena_map_locking::ekCACHES_HELD));

  map.maybe_lock(map.block_mtx(),
                 !(mc_locking & arena_map_locking::ekBLOCKS_HELD));

  /*
   * We might be modifying this cell--don't want block distribution in ANOTHER
   * thread to pick this cell for distribution.
   */
  map.maybe_lock(map.grid_mtx(), !(mc_locking & arena_map_locking::ekGRID_HELD));

  auto rloc = rmath::zvec2dvec(cell2D_op::coord(), mc_resolution.v());
  bool conflict = block_drop_loc_conflict(map, m_block, rloc);

  cds::cell2D& cell = map.access<arena_grid::kCell>(cell2D_op::coord());
  /*
   * Dropping a block onto a cell that already contains a single block (but not
   * a cache) does not work. Failing to do this results robots that are carrying
   * a block and that abort their current task causing the cell that they drop
   * the block onto to go into a HAS_CACHE state, when the cell entity is not a
   * cache.
   *
   * Dropping a block onto a cell that is part of a cache (CACHE_EXTENT), but
   * not the host cell doesn't work either (FSM state machine segfault), so we
   * need to drop the block in the host cell for the cache.
   */
  if (cell.state_has_cache() || cell.state_in_cache_extent()) {
    cache_block_drop_visitor op(m_block,
                                static_cast<carepr::arena_cache*>(cell.cache()),
                                mc_resolution,
                                arena_map_locking::ekALL_HELD);
    op.visit(map);
    map.maybe_unlock(map.cache_mtx(),
                     !(mc_locking & arena_map_locking::ekCACHES_HELD));
  } else if (cell.state_has_block() || conflict) {
    map.distribute_single_block(m_block, arena_map_locking::ekALL_HELD);
    map.maybe_unlock(map.cache_mtx(),
                     !(mc_locking & arena_map_locking::ekCACHES_HELD));
  } else {
    map.maybe_unlock(map.cache_mtx(),
                     !(mc_locking & arena_map_locking::ekCACHES_HELD));
    /*
     * Cell does not have a block/cache on it, so it is safe to drop the block
     * on it and change the cell state.
     *
     * Holding arena map grid lock, block lock if locking enabled.
     */
    visit(cell);

    /* set block extent */
    caops::block_extent_set_visitor e(m_block);
    e.visit(map.decoratee());
  }

  map.maybe_unlock(map.grid_mtx(),
                   !(mc_locking & arena_map_locking::ekGRID_HELD));
  map.maybe_unlock(map.block_mtx(),
                   !(mc_locking & arena_map_locking::ekBLOCKS_HELD));
} /* visit() */

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
bool block_drop_loc_conflict(const base_arena_map& map,
                             const crepr::base_block3D* const block,
                             const rmath::vector2d& loc) {
  /*
   * If the robot is currently right on the edge of a nest, we can't just drop
   * the block in it, as it will not be processed as a normal \ref
   * nest_block_drop, and will be discoverable by a robot via LOS but not able
   * to be acquired, as its color is hidden by that of the nest.
   */
  bool conflict = false;
  for (auto* nest : map.nests()) {
    conflict |= block_drop_overlap_with_nest(block, *nest, loc);
  } /* for(&nest..) */

  /*
   * If the robot is really close to a wall, then dropping a block may make it
   * inaccessible to future robots trying to reach it, due to obstacle avoidance
   * kicking in. This can result in an endless loop if said block is the only
   * one a robot knows about (see FORDYCA#242).
   */
  conflict |= map.distributable_areax().contains(loc.x()) &&
              map.distributable_areay().contains(loc.y());
  return conflict;
} /* block_drop_loc_conflict() */

bool block_drop_loc_conflict(const caching_arena_map& map,
                             const crepr::base_block3D* const block,
                             const rmath::vector2d& loc) {
  bool conflict = block_drop_loc_conflict(
      static_cast<const carena::base_arena_map&>(map), block, loc);

  /*
   * If the robot is currently right on the edge of a cache, we can't just drop
   * the block here, as it wipll overlap with the cache, and robots will think
   * that is accessible, but will not be able to vector to it (not all 4 wheel
   * sensors will report the color of a block). See FORDYCA#233.
   */
  for (auto& cache : map.caches()) {
    conflict |= block_drop_overlap_with_cache(block, cache, loc);
  } /* for(cache..) */
  return conflict;
} /* block_drop_loc_conflict() */

bool block_drop_overlap_with_nest(const crepr::base_block3D* const block,
                                  const crepr::nest& nest,
                                  const rmath::vector2d& drop_loc) {
  auto drop_xspan = crepr::entity3D::xrspan(drop_loc, block->xrsize());
  auto drop_yspan = crepr::entity3D::yrspan(drop_loc, block->yrsize());

  return nest.xrspan().overlaps_with(drop_xspan) &&
         nest.yrspan().overlaps_with(drop_yspan);
} /* block_drop_overlap_with_nest() */

bool block_drop_overlap_with_cache(const crepr::base_block3D* const block,
                                   const carepr::arena_cache* const cache,
                                   const rmath::vector2d& drop_loc) {
  auto drop_xspan = crepr::entity2D::xrspan(drop_loc, block->xrsize());
  auto drop_yspan = crepr::entity2D::yrspan(drop_loc, block->yrsize());
  return cache->xrspan().overlaps_with(drop_xspan) &&
         cache->yrspan().overlaps_with(drop_yspan);
} /* block_drop_overlap_with_cache() */

NS_END(detail, operations, arena, cosm);
