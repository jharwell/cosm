/**
 * \file robot_dynamics_applicator.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/tv/robot_dynamics_applicator.hpp"

#include "cosm/tv/config/robot_dynamics_applicator_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::tv {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
robot_dynamics_applicator::robot_dynamics_applicator(
    const tv::config::robot_dynamics_applicator_config* config)
    : ER_CLIENT_INIT("cosm.tv.robot_dynamics_applicator") {
  if (!config->motion_throttle.type.empty()) {
    m_motion_throttle_config = boost::make_optional(config->motion_throttle);
  }
  if (!config->block_carry_throttle.type.empty()) {
    m_bc_throttle_config = boost::make_optional(config->block_carry_throttle);
  }
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void robot_dynamics_applicator::register_controller(const rtypes::type_uuid& id) {
  if (!m_motion_throttle_config && !m_bc_throttle_config) {
    return;
  }
  if (m_motion_throttle_config) {
    m_motion_throttlers.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(id),
        std::forward_as_tuple(&m_motion_throttle_config.get()));
    m_motion_throttlers.at(id).toggle(true);
  }

  if (m_bc_throttle_config) {
    m_bc_throttlers.emplace(std::piecewise_construct,
                            std::forward_as_tuple(id),
                            std::forward_as_tuple(&m_bc_throttle_config.get()));
  }

  ER_INFO("Registered controller with ID=%d", id.v());
} /* register_controller() */

void robot_dynamics_applicator::unregister_controller(
    const rtypes::type_uuid& id) {
  if (!m_motion_throttle_config && !m_bc_throttle_config) {
    return;
  }

  if (m_motion_throttle_config) {
    m_motion_throttlers.erase(id);
  }

  if (m_bc_throttle_config) {
    m_bc_throttlers.erase(id);
  }
  ER_INFO("Unregistered controller with ID=%d", id.v());
} /* unregister_controller() */

} /* namespace cosm::tv */
