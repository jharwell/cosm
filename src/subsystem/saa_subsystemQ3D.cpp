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
  auto odom = odometry();
  ER_DEBUG("position=%s azimuth=%s zenith=%s",
           rcppsw::to_string(odom.pose.position).c_str(),
           rcppsw::to_string(odom.pose.orientation.z()).c_str(),
           rcppsw::to_string(odom.pose.orientation.y()).c_str());
  ER_DEBUG("twist.linear=%s@%s [%s] twist.angular=%s [%s]",
           rcppsw::to_string(odom.twist.linear).c_str(),
           rcppsw::to_string(odom.twist.linear.to_2D().angle()).c_str(),
           rcppsw::to_string(odom.twist.linear.length()).c_str(),
           rcppsw::to_string(odom.twist.angular).c_str(),
           rcppsw::to_string(odom.twist.angular.length()).c_str());

  ER_DEBUG("steering_force=%s@%s [%s]",
           rcppsw::to_string(steer_force2D().value()).c_str(),
           rcppsw::to_string(steer_force2D().value().angle()).c_str(),
           rcppsw::to_string(steer_force2D().value().length()).c_str());
  RCPPSW_UNUSED double applied =
      actuation()->governed_diff_drive()->applied_throttle();
  double active = actuation()->governed_diff_drive()->active_throttle();
  ER_DEBUG("Applied throttle: %f active throttle: %f", applied, active);

  auto force = steer_force2D().value();
  auto desired_speed = force.length() * (1.0 - active);
  ckin::twist delta;
  delta.linear = rmath::vector3d::X * desired_speed;
  delta.angular = rmath::vector3d::Z * force.angle().v();
  actuation()->governed_diff_drive()->fsm_drive(delta);

  steer_force2D().forces_reset();
} /* steer_force2D_apply() */

ckin::odometry saa_subsystemQ3D::odometry(void) const {
  return sensing()->odometry()->reading();
} /* odometry() */

double saa_subsystemQ3D::max_speed(void) const {
  return actuation()->governed_diff_drive()->max_speed();
} /* max_speed() */

NS_END(subsystem, cosm);
