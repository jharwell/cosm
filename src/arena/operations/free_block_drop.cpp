/**
 * \file free_block_drop.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/arena/operations/free_block_drop.hpp"

#include "rcppsw/utils/maskable_enum.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/arena/operations/block_extent_set.hpp"
#include "cosm/arena/operations/cache_block_drop.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/ds/operations/cell2D_block_extent.hpp"
#include "cosm/foraging/block_dist/base_distributor.hpp"
#include "cosm/repr/sim_block3D.hpp"
#include "cosm/spatial/common/conflict_checker.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::operations::detail {
using cads::arena_grid;

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
free_block_drop
free_block_drop::for_block(const rmath::vector2z& coord,
                           const rtypes::discretize_ratio& resolution) {
  return free_block_drop(nullptr, coord, resolution, locking::ekNONE_HELD);
} /* for_block() */

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
free_block_drop::free_block_drop(crepr::sim_block3D* block,
                                 const rmath::vector2z& coord,
                                 const rtypes::discretize_ratio& resolution,
                                 const locking& locking)
    : ER_CLIENT_INIT("cosm.arena.operations.free_block_drop"),
      cell2D_op(coord),
      mc_resolution(resolution),
      mc_locking(locking),
      m_block(block) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void free_block_drop::visit(cds::cell2D& cell) {
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

void free_block_drop::visit(crepr::sim_block3D& block) {
  /* block is no longer carried by a robot */
  block.md()->robot_id_reset();

  /* update block location */
  auto rloc =
      rmath::vector3d(rmath::zvec2dvec(cell2D_op::coord(), mc_resolution.v()));

  block.ranchor3D(rloc);
  block.danchor3D(rmath::vector3z(cell2D_op::coord()));
} /* visit() */

void free_block_drop::visit(base_arena_map& map) {
  /*
   * We might need all locks:
   *
   * - Grid+block to avoid ANOTHER thread to pick this cell for block
   *   distribution.
   */
  map.ordered_lock(mc_locking);

  cds::cell2D& cell = map.access<arena_grid::kCell>(cell2D_op::coord());

  auto rloc = rmath::zvec2dvec(cell2D_op::coord(), mc_resolution.v());
  auto status = cspatial::conflict_checker::placement2D(&map, m_block, rloc);
  bool conflict = status.x && status.y;
  /*
   * Dropping a block onto a cell that already contains a single block (but not
   * a cache) does not work. Failing to do this results robots that are carrying
   * a block and that abort their current task causing the cell that they drop
   * the block onto to go into a HAS_CACHE state, when the cell entity is not a
   * cache.
   */
  if (cell.state_has_block() || cell.state_in_block_extent() || conflict) {
    ER_DEBUG("Free drop of block%s@%s not possible: cell_state=%d,conflict=%d -> "
             "redistribute",
             rcppsw::to_string(m_block->id()).c_str(),
             rcppsw::to_string(coord()).c_str(),
             cell.fsm().current_state(),
             conflict);
    map.distribute_single_block(m_block, locking::ekALL_HELD);
  } else {
    /*
     * Cell does not have a block/cache on it, so it is safe to drop the block
     * on it and change the cell state.
     *
     * Holding arena map grid lock, block lock if locking enabled.
     */
    execute_free_drop(map, cell);
  }
  map.ordered_unlock(mc_locking);
} /* visit() */

void free_block_drop::visit(caching_arena_map& map) {
  /*
   * We might need all locks:
   *
   * - Cache+block for atomic check for cache overlap+do drop operation
   * - Grid for updating cells
   */
  map.ordered_lock(mc_locking);

  cds::cell2D& cell = map.access<arena_grid::kCell>(cell2D_op::coord());

  auto rloc = rmath::zvec2dvec(cell2D_op::coord(), mc_resolution.v());
  auto status = cspatial::conflict_checker::placement2D(&map, m_block, rloc);
  bool conflict = status.x && status.y;

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
    ER_DEBUG("Free drop of block%s@%s resolves to cache block drop",
             rcppsw::to_string(m_block->id()).c_str(),
             rcppsw::to_string(coord()).c_str());
    cache_block_drop_visitor op(m_block,
                                static_cast<carepr::arena_cache*>(cell.cache()),
                                mc_resolution,
                                locking::ekALL_HELD);
    op.visit(map);
  } else if (cell.state_has_block() || cell.state_in_block_extent() || conflict) {
    ER_DEBUG("Free drop of block%s@%s invalid: cell_state=%d,conflict=%d -> "
             "redistribute",
             rcppsw::to_string(m_block->id()).c_str(),
             rcppsw::to_string(m_block->danchor2D()).c_str(),
             cell.fsm().current_state(),
             conflict);
    map.distribute_single_block(m_block, locking::ekALL_HELD);
  } else {
    /*
     * Cell does not have a block/cache on it, so it is safe to drop the block
     * on it and change the cell state.
     *
     * Holding arena map grid lock, block lock if locking enabled.
     */
    execute_free_drop(map, cell);
  }
  map.ordered_unlock(mc_locking);
} /* visit() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
template <typename TMap>
void free_block_drop::execute_free_drop(TMap& map, cds::cell2D& cell) {
  ER_DEBUG("Execute free drop of block%s@%s",
           rcppsw::to_string(m_block->id()).c_str(),
           rcppsw::to_string(coord()).c_str());

  visit(*m_block);
  visit(cell);

  /* set block extent */
  caops::block_extent_set_visitor e(m_block);
  e.visit(map.decoratee());

  /* update block loctree with new location */
  map.bloctree_update(m_block, locking::ekALL_HELD);

  /* possibly update block cluster membership */
  map.block_distributor()->cluster_update_after_drop(m_block);
} /* execute_free_drop() */

/*******************************************************************************
 * Template Instantiations
 ******************************************************************************/
template void free_block_drop::execute_free_drop(carena::base_arena_map&,
                                                 cds::cell2D&);
template void free_block_drop::execute_free_drop(carena::caching_arena_map&,
                                                 cds::cell2D&);

} /* namespace cosm::arena::operations::detail */
