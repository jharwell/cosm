/**
 * \file footbot_saa_subsystem.cpp
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
#include "cosm/robots/footbot/footbot_saa_subsystem.hpp"

#include "cosm/subsystem/actuation_subsystem2D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, robots, footbot);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
footbot_saa_subsystem::footbot_saa_subsystem(
    const hal::sensors::position_sensor& pos,
    const subsystem::sensing_subsystemQ3D::sensor_map& sensors,
    const subsystem::actuation_subsystem2D::actuator_map& actuators,
    const steer2D::config::force_calculator_config* const steer_config)
    : saa_subsystemQ3D(pos, sensors, actuators, steer_config),
      ER_CLIENT_INIT("cosm.robots.footbot.saa") {
  /*
   * Before this, the sensing member of our parent class is of the wrong type
   * for the static casts we do.
   */
  auto footbot_sensing = std::make_unique<footbot_sensing_subsystem>(pos,
                                                                     sensors);
  saa_subsystemQ3D::sensing(std::move(footbot_sensing));
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void footbot_saa_subsystem::steer_force2D_apply(void) {
  ER_DEBUG("position=%s azimuth=%s zenith=%s",
           sensing()->rpos3D().to_str().c_str(),
           sensing()->azimuth().to_str().c_str(),
           sensing()->zenith().to_str().c_str());
  ER_DEBUG("linear_vel=%s@%s [%f] angular_vel=%f",
           linear_velocity().to_str().c_str(),
           linear_velocity().angle().to_str().c_str(),
           linear_velocity().length(),
           angular_velocity());
  ER_DEBUG("steering_force=%s@%s [%f]",
           steer_force2D().value().to_str().c_str(),
           steer_force2D().value().angle().to_str().c_str(),
           steer_force2D().value().length());
  RCSW_UNUSED double applied = actuation()->governed_diff_drive()->applied_throttle();
  double active = actuation()->governed_diff_drive()->active_throttle();
  ER_DEBUG("Applied throttle: %f active throttle: %f", applied, active);

  double desired_speed = steer_force2D().value().length() * (1.0 - active);
  actuation()->governed_diff_drive()->fsm_drive(desired_speed,
                                                steer_force2D().value().angle());

  steer_force2D().forces_reset();
} /* steer_force2D_apply() */

rmath::vector2d footbot_saa_subsystem::linear_velocity(void) const {
  auto speed = sensing()->diff_drive()->current_speed();
  /*
   * If speed comes back as 0.0, then we are executing a hard turn, probably as
   * we vector somewhere. In order to have the arrival force work properly, we
   * need to have a velocity with a non-zero length and the correct heading
   * angle at all times. So we report that we have velocity even though we do
   * not, for the purposes of making those calculations work.
   *
   * There probably is a better way to do this, but I don't know what it is.
   */
  if (speed <= std::numeric_limits<double>::epsilon()) {
    return {0.1, sensing()->azimuth()};
  } else {
    return {sensing()->diff_drive()->current_speed(), sensing()->azimuth()};
  }
} /* linear_velocity() */

double footbot_saa_subsystem::angular_velocity(void) const {
  auto reading = sensing()->diff_drive()->reading();

  return (reading.vel_right - reading.vel_left) / reading.axle_length;
} /* angular_velocity() */

double footbot_saa_subsystem::max_speed(void) const {
  return actuation()->governed_diff_drive()->max_speed();
} /* max_speed() */

rmath::vector2d footbot_saa_subsystem::pos2D(void) const {
  return sensing()->rpos2D();
} /* pos2D() */

NS_END(footbot, robots, cosm);
