/**
 * \file diff_drive.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "cosm/kin2D/diff_drive.hpp"

#include "cosm/kin2D/config/diff_drive_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, kin2D);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
diff_drive::diff_drive(const config::diff_drive_config* const config,
                       const hal::actuators::diff_drive_actuator& actuator,
                       drive_type type)
    : ER_CLIENT_INIT("cosm.kin2D.diff_drive"),
      m_drive_type(type),
      m_max_speed(config->max_speed),
      m_fsm(config->max_speed, config->soft_turn_max),
      m_actuator(actuator) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
status_t diff_drive::fsm_drive(double desired_speed,
                               const rmath::radians& angle_delta) {
  std::pair<double, double> speeds;
  ER_CHECK(drive_type::ekFSM_DRIVE == m_drive_type,
           "Cannot actuate: not in FSM drive mode");
  m_fsm.change_velocity(desired_speed, angle_delta);
  speeds = m_fsm.wheel_speeds();

  /* don't need to normalize--done by fsm internally */
  m_actuator.set_wheel_speeds(speeds.first, speeds.second);
  return OK;

error:
  return ERROR;
} /* fsm_drive() */

status_t diff_drive::tank_drive(double left_speed,
                                double right_speed,
                                bool square_inputs) {
  ER_CHECK(drive_type::ekTANK_DRIVE == m_drive_type,
           "Cannot actuate: not in tank drive mode");

  if (square_inputs) {
    left_speed = std::copysign(std::pow(left_speed, 2), left_speed);
    right_speed = std::copysign(std::pow(right_speed, 2), right_speed);
  }

  m_actuator.set_wheel_speeds(left_speed, right_speed);
  return OK;

error:
  return ERROR;
} /* tank_drive() */

double diff_drive::limit(double value) const {
  return std::max(std::min(value, m_max_speed), -m_max_speed);
} /* limit() */

NS_END(kin2D, cosm);
