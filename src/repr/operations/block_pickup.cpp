/**
 * \file block_pickup.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/repr/operations/block_pickup.hpp"

#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::repr::operations {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_pickup::block_pickup(const rtypes::type_uuid& robot_id,
                           const rtypes::timestep& t)
    : ER_CLIENT_INIT("cosm.repr.operations.block_pickup"),
      mc_timestep(t),
      mc_robot_id(robot_id) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_pickup::visit(crepr::base_block3D& block,
                         const block_pickup_owner& owner) {
  block.update_on_pickup(mc_robot_id, mc_timestep, owner);
} /* visit() */

} /* namespace cosm::repr::operations */
