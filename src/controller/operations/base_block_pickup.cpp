/**
 * \file base_block_pickup.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
      mc_timestep(t),
      mc_robot_id(robot_id),
      m_block(block) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void base_block_pickup::visit(controller::block_carrying_controller& c) {
  ER_ASSERT(
      !c.is_carrying_block(), "Robot%u already carrying block!", mc_robot_id.v());

  /*
   * Cloning resets robot ID, so we need to set it again in the clone (principle
   * of least surprise).
   */
  auto block = m_block->clone();
  block->md()->robot_id(mc_robot_id);
  c.block(std::move(block));
  ER_INFO("Block%d@%s",
          m_block->id().v(),
          rcppsw::to_string(m_block->danchor2D()).c_str());

  /*
   * We need to visit the cloned controller block, because metrics are collected
   * from it, and we need to update its state so they are collected correctly.
   */
  crops::block_pickup op(mc_robot_id, mc_timestep);
  op.visit(*c.block(), crops::block_pickup_owner::ekROBOT);
} /* visit() */

NS_END(operations, controller, cosm);
