/**
 * \file block_pickup.cpp
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
#include "cosm/controller/operations/block_pickup.hpp"

#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, controller, operations);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_pickup::block_pickup(crepr::base_block3D* block,
                           const rtypes::type_uuid& robot_id,
                           const rtypes::timestep& t)
    : ER_CLIENT_INIT("cosm.controller.operations.block_pickup"),
      cell2D_op(block->dpos2D()),
      mc_timestep(t),
      mc_robot_id(robot_id),
      m_block(block) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_pickup::visit(controller::block_carrying_controller& c) {
  /*
   * Cloning resets robot ID, so we need to set it again in the clone (principle
   * of least surprise).
   */
  auto block = m_block->clone();
  block->md()->robot_id(mc_robot_id);
  c.block(std::move(block));
  ER_INFO("Block%d is now carried by fb%u", m_block->id().v(), mc_robot_id.v());


  /*
   * We need to visit the cloned controller block rather than the one we are
   * passed in the constructor, because THAT one is owned by the arena map, and
   * metrics are collected on the block the robot is carrying upon drop.
   */
  visit(*c.block());

  /* move the arena map block out of sight */
  m_block->robot_pickup_update(mc_robot_id,
                               mc_timestep,
                               crepr::base_block3D::pickup_owner::ekARENA_MAP);
} /* visit() */

void block_pickup::visit(crepr::base_block3D& block) {
  ER_ASSERT(rtypes::constants::kNoUUID != block.id(), "Unamed block");
  block.robot_pickup_update(mc_robot_id,
                            mc_timestep,
                            crepr::base_block3D::pickup_owner::ekROBOT);
} /* visit() */

NS_END(operations, controller, cosm);
