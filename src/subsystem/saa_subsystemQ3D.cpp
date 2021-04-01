/**
 * \file saa_subsystemQ3D.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include "cosm/subsystem/saa_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
saa_subsystemQ3D::saa_subsystemQ3D(
    const chsubsystem::sensing_subsystemQ3D::sensor_map& sensors,
    const chsubsystem::actuation_subsystem2D::actuator_map& actuators,
    const steer2D::config::force_calculator_config* const steer_config)
    : ER_CLIENT_INIT("cosm.subsystem.saa_subsystemQ3D"),
      m_actuation(std::make_unique<actuation_type>(actuators)),
      m_sensing(std::make_unique<sensing_type>(sensors)),
      m_steer2D_calc(*this, steer_config) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void saa_subsystemQ3D::steer_force2D_apply(void) {
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
  RCPPSW_UNUSED double applied =
      actuation()->governed_diff_drive()->applied_throttle();
  double active = actuation()->governed_diff_drive()->active_throttle();
  ER_DEBUG("Applied throttle: %f active throttle: %f", applied, active);

  double desired_speed = steer_force2D().value().length() * (1.0 - active);
  actuation()->governed_diff_drive()->fsm_drive(desired_speed,
                                                steer_force2D().value().angle());

  steer_force2D().forces_reset();
} /* steer_force2D_apply() */

rmath::vector2d saa_subsystemQ3D::linear_velocity(void) const {
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
    return { 0.1, sensing()->azimuth() };
  } else {
    return { sensing()->diff_drive()->current_speed(), sensing()->azimuth() };
  }
} /* linear_velocity() */

double saa_subsystemQ3D::angular_velocity(void) const {
  auto reading = sensing()->diff_drive()->reading();

  return (reading.vel_right - reading.vel_left) / reading.axle_length;
} /* angular_velocity() */

double saa_subsystemQ3D::max_speed(void) const {
  return actuation()->governed_diff_drive()->max_speed();
} /* max_speed() */

rmath::vector2d saa_subsystemQ3D::pos2D(void) const {
  return sensing()->rpos2D();
} /* pos2D() */

NS_END(subsystem, cosm);
