/**
 * \file footbot_actuation_subsystem.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_ROBOTS_FOOTBOT_FOOTBOT_ACTUATION_SUBSYSTEM_HPP_
#define INCLUDE_COSM_ROBOTS_FOOTBOT_FOOTBOT_ACTUATION_SUBSYSTEM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/subsystem/actuation_subsystem2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, robots, footbot);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class footbot_actuation_subsystem
 * \ingroup robots footbot
 *
 * \brief The 2D actuation subsystem for the foot-bot robot.
 */
class footbot_actuation_subsystem : public subsystem::actuation_subsystem2D {
 public:
  explicit footbot_actuation_subsystem(actuator_map& actuators)
      : actuation_subsystem2D(actuators) {}

  kin2D::governed_diff_drive* governed_diff_drive(void) {
    return actuator<kin2D::governed_diff_drive>();
  }

  const kin2D::governed_diff_drive* governed_diff_drive(void) const {
    return actuator<kin2D::governed_diff_drive>();
  }

  hal::actuators::led_actuator* leds(void) {
    return actuator<hal::actuators::led_actuator>();
  }

  const hal::actuators::led_actuator* leds(void) const {
    return actuator<hal::actuators::led_actuator>();
  }

  hal::actuators::wifi_actuator* rab(void) {
    return actuator<hal::actuators::wifi_actuator>();
  }

  const hal::actuators::wifi_actuator* rab(void) const {
    return actuator<hal::actuators::wifi_actuator>();
  }
};

NS_END(footbot, controller, cosm);

#endif /* INCLUDE_COSM_ROBOTS_FOOTBOT_FOOTBOT_ACTUATION_SUBSYSTEM_HPP_ */
