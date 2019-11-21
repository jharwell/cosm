/**
 * \file swarm_irv_manager.cpp
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
#include "cosm/tv/swarm_irv_manager.hpp"

#include "cosm/tv/config/swarm_irv_manager_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, tv);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
swarm_irv_manager::swarm_irv_manager(
    const tv::config::swarm_irv_manager_config* config)
    : ER_CLIENT_INIT("cosm.tv.swarm_irv_manager") {
  if (!config->motion_throttle.type.empty()) {
    mc_motion_throttle_config = boost::make_optional(config->motion_throttle);
  }
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void swarm_irv_manager::register_controller(int robot_id) {
  if (mc_motion_throttle_config) {
    m_motion_throttling.emplace(std::piecewise_construct,
                                std::forward_as_tuple(robot_id),
                                std::forward_as_tuple(
                                    &mc_motion_throttle_config.get()));
  }
} /* register_controller() */

NS_END(tv, cosm);
