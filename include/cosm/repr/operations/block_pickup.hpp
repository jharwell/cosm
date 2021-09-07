/**
 * \file block_pickup.hpp
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

#ifndef INCLUDE_COSM_REPR_OPERATIONS_BLOCK_PICKUP_HPP_
#define INCLUDE_COSM_REPR_OPERATIONS_BLOCK_PICKUP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/types/type_uuid.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/repr/operations/block_pickup_owner.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class base_block3D;
} /* namespace cosm::repr */

NS_START(cosm, repr, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_pickup
 * \ingroup repr operations
 *
 * \brief Handles updates to block state whenever a block is picked up by a
 * robot, for both the arena map owned block, and the clone owned by the robot.
 */
class block_pickup : public rer::client<block_pickup> {
 public:
  /**
   * \param robot_id ID of the robot doing the pickup.
   * \param t The current timestep.
   */
  block_pickup(const rtypes::type_uuid& robot_id,
               const rtypes::timestep& t);

  virtual ~block_pickup(void) = default;

  /* not copy constructible/assignable by default */
  block_pickup(const block_pickup&) = delete;
  block_pickup& operator=(const block_pickup&) = delete;

  void visit(crepr::base_block3D& block,
             const block_pickup_owner& owner);

 private:
  /* clang-format off */
  const rtypes::timestep  mc_timestep;
  const rtypes::type_uuid mc_robot_id;
  /* clang-format on */
};

NS_END(operations, repr, cosm);

#endif /* INCLUDE_COSM_REPR_OPERATIONS_BLOCK_PICKUP_HPP_ */
