/**
 * \file free_block_pickup.cpp
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
#include "cosm/arena/operations/free_block_pickup.hpp"

#include "rcppsw/utils/maskable_enum.hpp"

#include "cosm/arena/base_arena_map.hpp"
#include "cosm/arena/operations/block_extent_clear.hpp"
#include "cosm/ds/operations/cell2D_empty.hpp"
#include "cosm/foraging/block_dist/base_distributor.hpp"
#include "cosm/foraging/repr/block_cluster.hpp"
#include "cosm/repr/operations/block_pickup.hpp"
#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, operations, detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
free_block_pickup free_block_pickup::by_robot(crepr::sim_block3D* block,
                                              const rtypes::type_uuid& robot_id,
                                              const rtypes::timestep& t,
                                              const locking& locking) {
  return free_block_pickup(block, robot_id, t, locking);
} /* by_robot() */

free_block_pickup free_block_pickup::by_arena(crepr::sim_block3D* block) {
  /*
   * Passing "NO UUID" as the ID of the robot which picked up the arena map's
   * copy of a block works fine, as long as it is a robot which triggers the
   * free pickup event. If it is triggered by block motion then the pickup event
   * does everything it should EXCEPT moving the block out of sight.
   *
   * This is OK, because the block is immediately "dropped" again on its new
   * location, and would no longer be out of sight anyway.
   */
  return free_block_pickup(block,
                           rtypes::constants::kNoUUID,
                           rtypes::constants::kNoTime,
                           locking::ekALL_HELD);
} /* by_arena() */

free_block_pickup::free_block_pickup(crepr::sim_block3D* block,
                                     const rtypes::type_uuid& robot_id,
                                     const rtypes::timestep& t,
                                     const locking& locking)
    : ER_CLIENT_INIT("cosm.arena.operations.free_block_pickup"),
      cell2D_op(block->danchor2D()),
      mc_robot_id(robot_id),
      mc_timestep(t),
      mc_locking(locking),
      m_block(block) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void free_block_pickup::visit(cads::arena_grid& grid) {
  ER_ASSERT(!m_block->is_out_of_sight(),
            "Block%d out of sight on pickup",
            m_block->id().v());

  caops::block_extent_clear_visitor ec(m_block);
  cdops::cell2D_empty_visitor hc(coord());

  if (!(mc_locking & locking::ekGRID_HELD)) {
    grid.mtx()->lock();
  }

  /* mark host cell as empty (not done as part of clearing block extent) */
  hc.visit(grid);

  /* clear block extent */
  ec.visit(grid);

  if (!(mc_locking & locking::ekGRID_HELD)) {
    grid.mtx()->unlock();
  }

  if (rtypes::constants::kNoUUID != mc_robot_id) {
    /* Update block state--already holding block mutex if it is needed */
    crops::block_pickup block_op(mc_robot_id, mc_timestep);
    block_op.visit(*m_block, crops::block_pickup_owner::ekARENA_MAP);
  }
} /* visit() */

void free_block_pickup::visit(base_arena_map& map) {
  /* capture where the block used to be */
  rmath::vector2z old = m_block->danchor2D();

  /* update the arena grid */
  visit(map.decoratee());

  /* update block loctree */
  map.bloctree_update(m_block, mc_locking);

  /*
   * Update block clusters--the picked up block MIGHT have disappeared from one
   * of them. This could also be a free pickup from a block that was previously
   * dropped as the result of a task abort, block motion, etc., and not be in a
   * cluster.
   */
  map.block_distributor()->cluster_update_after_pickup(m_block, old);
} /* visit() */

NS_END(detail, operations, arena, cosm);
