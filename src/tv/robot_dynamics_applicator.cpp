/**
 * \file robot_dynamics_applicator.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "cosm/tv/robot_dynamics_applicator.hpp"

#include "cosm/tv/config/robot_dynamics_applicator_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, tv);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
robot_dynamics_applicator::robot_dynamics_applicator(
    const tv::config::robot_dynamics_applicator_config* config)
    : ER_CLIENT_INIT("cosm.tv.robot_dynamics_applicator") {
  if (!config->motion_throttle.type.empty()) {
    mc_motion_throttle_config = boost::make_optional(config->motion_throttle);
  }
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void robot_dynamics_applicator::register_controller(const rtypes::type_uuid& id) {
  if (mc_motion_throttle_config) {
    m_motion_throttlers.emplace(std::piecewise_construct,
                                std::forward_as_tuple(id),
                                std::forward_as_tuple(
                                    &mc_motion_throttle_config.get()));
    ER_INFO("Registered controller with ID=%d", id.v());
  }
} /* register_controller() */

void robot_dynamics_applicator::unregister_controller(const rtypes::type_uuid& id) {
  if (mc_motion_throttle_config) {
    m_motion_throttlers.erase(id);
    ER_INFO("Unregistered controller with ID=%d", id.v());
  }
} /* unregister_controller() */

NS_END(tv, cosm);
