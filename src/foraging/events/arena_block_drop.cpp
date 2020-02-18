/**
 * \file arena_block_drop.cpp
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
#include "cosm/foraging/events/arena_block_drop.hpp"

#include "cosm/ds/cell2D.hpp"
#include "cosm/foraging/ds/arena_map.hpp"
#include "cosm/foraging/events/arena_cache_block_drop.hpp"
#include "cosm/foraging/repr/arena_cache.hpp"
#include "cosm/repr/base_block2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, events);
using cds::arena_grid;

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
arena_block_drop arena_block_drop::for_block(const rmath::vector2u& coord,
                                             const rtypes::discretize_ratio& resolution) {
  return arena_block_drop(nullptr, coord, resolution, false);
} /* for_block() */

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
arena_block_drop::arena_block_drop(const std::shared_ptr<crepr::base_block2D>& block,
                                   const rmath::vector2u& coord,
                                   const rtypes::discretize_ratio& resolution,
                                   bool cache_lock)
    : ER_CLIENT_INIT("cosm.foraging.events.arena_block_drop"),
      cell2D_op(coord),
      mc_resolution(resolution),
      mc_cache_lock(cache_lock),
      m_block(block) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void arena_block_drop::visit(cds::cell2D& cell) {
  visit(*m_block);
  visit(cell.fsm());
  cell.entity(m_block);
} /* visit() */

void arena_block_drop::visit(fsm::cell2D_fsm& fsm) {
  fsm.event_block_drop();
} /* visit() */

void arena_block_drop::visit(crepr::base_block2D& block) {
  block.reset_robot_id();

  block.rloc(rmath::uvec2dvec(cell2D_op::coord(), mc_resolution.v()));
  block.dloc(cell2D_op::coord());
} /* visit() */

void arena_block_drop::visit(cfds::arena_map& map) {
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
   *
   * 2020/2/21: Tweak logic so that if the free block drop is outside the
   * distributable area for the arena (which can only happen if a robot aborts
   * its task near the arena walls while carrying a block) the block is
   * distributed rather than just dropped. This can cause problems in large
   * simulations with groups robots tracking blocks too close to the arena edge
   * that they ALL try to acquire, which can in some instances result in one or
   * more of them getting pushed outside of physics engine boundaries.
   *
   * In that case, distribute the block somewhere within the "safe"
   * distributable area.
   */
  bool ok_to_drop = map.distributable_areax().contains(cell2D_op::coord().x()) &&
                 map.distributable_areay().contains(cell2D_op::coord().y());
  if (cell.state_has_cache() || cell.state_in_cache_extent()) {
    if (mc_cache_lock) {
      map.cache_mtx().lock();
    }
    arena_cache_block_drop_visitor op(m_block,
                                      std::static_pointer_cast<cfrepr::arena_cache>(
                                          cell.cache()),
                                      mc_resolution);
    op.visit(map);
    if (mc_cache_lock) {
      map.cache_mtx().unlock();
    }
  } else if (cell.state_has_block() || !ok_to_drop) {
    map.distribute_single_block(m_block);
  } else {
    /*
     * Cell does not have a block/cache on it, so it is safe to drop the block
     * on it and change the cell state.
     */
    visit(cell);
  }
} /* visit() */

NS_END(events, foraging, cosm);
