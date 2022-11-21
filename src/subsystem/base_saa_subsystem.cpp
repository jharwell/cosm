/**
 * \file base_saa_subsystem.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/subsystem/base_saa_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
base_saa_subsystem::base_saa_subsystem(sensor_map&& sensors,
                                       actuator_map&& actuators,
                                       const apf2D::config::apf_manager_config* const apf_config)
    : ER_CLIENT_INIT("cosm.subsystem.base_saa_subsystem"),
      m_actuation(std::make_unique<actuation_type>(std::move(actuators))),
      m_sensing(std::make_unique<sensing_type>(std::move(sensors))),
      m_apf(*this, apf_config) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
ckin::odometry base_saa_subsystem::odometry(void) const {
  return sensing()->odometry()->reading();
}

double base_saa_subsystem::max_velocity(void) const {
  return actuation()->locomotion()->max_velocity();
}

} /* namespace cosm::subsystem */
