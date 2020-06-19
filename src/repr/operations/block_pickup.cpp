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
#include "cosm/repr/operations/block_pickup.hpp"

#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr, operations);

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
  ER_ASSERT(rtypes::constants::kNoUUID != block.id(), "Unamed block");
  block.update_on_pickup(mc_robot_id, mc_timestep, owner);
} /* visit() */

NS_END(operations, repr, cosm);
