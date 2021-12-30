/**
 * \file base_block_pickup.hpp
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

#ifndef INCLUDE_COSM_CONTROLLER_OPERATIONS_BASE_BLOCK_PICKUP_HPP_
#define INCLUDE_COSM_CONTROLLER_OPERATIONS_BASE_BLOCK_PICKUP_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/types/type_uuid.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/controller/block_carrying_controller.hpp"
#include "cosm/ds/operations/cell2D_op.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, controller, operations);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_block_pickup
 * \ingroup controller operations
 *
 * \brief Base class for the simulation event fired whenever a robot picks up a
 * block. Has "base" in the name, because it does not contain the necessary
 * functions for updating the FSM attached to a controller, which is necessary
 * for it to be reused as-is, hence it is therefore a base class.
 */
class base_block_pickup : public rer::client<base_block_pickup>,
                          public cdops::cell2D_op {
 public:
  /**
   * \param block Non-owning reference to the block to be picked up; block is
   *              owned by arena map.
   *
   * \param robot_id ID of the robot doing the pickup.
   *
   * \param t The current timestep.
   */

  base_block_pickup(crepr::base_block3D* block,
                    const rtypes::type_uuid& robot_id,
                    const rtypes::timestep& t);

  virtual ~base_block_pickup(void) = default;

  /* not copy constructible/assignable by default */
  base_block_pickup(const base_block_pickup&) = delete;
  base_block_pickup& operator=(const base_block_pickup&) = delete;

  /**
   * \brief Update the controller with the block it has just picked up, along
   * with updating necessary block metadata to reflect the fact that it is now
   * carried by a robot.
   */
  void visit(controller::block_carrying_controller& controller);

  crepr::base_block3D* block(void) const { return m_block; }
  const rtypes::type_uuid& robot_id(void) const { return mc_robot_id; }

 private:
  /* clang-format off */
  const rtypes::timestep  mc_timestep;
  const rtypes::type_uuid mc_robot_id;

  crepr::base_block3D*    m_block;
  /* clang-format on */
};

NS_END(operations, controller, cosm);

#endif /* INCLUDE_COSM_CONTROLLER_OPERATIONS_BASE_BLOCK_PICKUP_HPP_ */
