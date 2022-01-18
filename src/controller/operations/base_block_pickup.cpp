/**
 * \file base_block_pickup.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
#include "cosm/controller/operations/base_block_pickup.hpp"

#include "cosm/repr/base_block3D.hpp"
#include "cosm/repr/operations/block_pickup.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, controller, operations);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_block_pickup::base_block_pickup(crepr::base_block3D* block,
                                     const rtypes::type_uuid& robot_id,
                                     const rtypes::timestep& t)
    : ER_CLIENT_INIT("cosm.controller.operations.base_block_pickup"),
      cell2D_op(block->danchor2D()),
      mc_timestep(t),
      mc_robot_id(robot_id),
      m_block(block) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_block_pickup::visit(controller::block_carrying_controller& c) {
  ER_ASSERT(!c.is_carrying_block(),
            "Robot%u already carrying block!",
            mc_robot_id.v());

  /*
   * Cloning resets robot ID, so we need to set it again in the clone (principle
   * of least surprise).
   */
  auto block = m_block->clone();
  block->md()->robot_id(mc_robot_id);
  c.block(std::move(block));
  ER_INFO("Block%d", m_block->id().v());

  /*
   * We need to visit the cloned controller block, because metrics are collected
   * from it, and we need to update its state so they are collected correctly.
   */
  crops::block_pickup op(mc_robot_id, mc_timestep);
  op.visit(*c.block(), crops::block_pickup_owner::ekROBOT);
} /* visit() */

NS_END(operations, controller, cosm);
