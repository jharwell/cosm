/**
 * \file saa_subsystemQ3D.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/subsystem/saa_subsystemQ3D.hpp"

#include "cosm/subsystem/actuation_subsystem2D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
saa_subsystemQ3D::saa_subsystemQ3D(
    chsubsystem::sensor_variant_map<COSM_HAL_ROBOT_AVAILABLE_SENSORS>&& sensors,
    chsubsystem::actuator_variant_map<COSM_HAL_ROBOT_AVAILABLE_ACTUATORS>&&
        actuators,
    const apf2D::config::apf_manager_config* const apf_config)
    : ER_CLIENT_INIT("cosm.subsystem.saa_subsystemQ3D"),
      m_actuation(std::make_unique<actuation_type>(std::move(actuators))),
      m_sensing(std::make_unique<sensing_type>(std::move(sensors))),
      m_apf2D(*this, apf_config) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void saa_subsystemQ3D::apf2D_apply(void) {
  if (!apf2D().is_enabled()) {
    ER_DEBUG("Skipping applying steering forces--disabled");
    return;
  }
  auto odom = odometry();
  ER_DEBUG("odom.position=%s azimuth=%s zenith=%s",
           rcppsw::to_string(odom.pose.position).c_str(),
           rcppsw::to_string(odom.pose.orientation.z()).c_str(),
           rcppsw::to_string(odom.pose.orientation.y()).c_str());
  ER_DEBUG("odom.twist.linear=%s@%s [%s]",
           rcppsw::to_string(odom.twist.linear).c_str(),
           rcppsw::to_string(odom.twist.linear.to_2D().angle()).c_str(),
           rcppsw::to_string(odom.twist.linear.length()).c_str());
  ER_DEBUG("odom.twist.angular=%s [%s]",
           rcppsw::to_string(odom.twist.angular).c_str(),
           rcppsw::to_string(odom.twist.angular.length()).c_str());

  ER_DEBUG("steering_force=%s@%s [%s]",
           rcppsw::to_string(apf2D().value()).c_str(),
           rcppsw::to_string(apf2D().value().angle()).c_str(),
           rcppsw::to_string(apf2D().value().length()).c_str());
  RCPPSW_UNUSED double applied =
      actuation()->governed_diff_drive()->applied_throttle();
  double active = actuation()->governed_diff_drive()->active_throttle();
  ER_DEBUG("Applied throttle: %f active throttle: %f", applied, active);

  auto force = apf2D().value();
  auto desired_speed = force.length() * (1.0 - active);
  ckin::twist delta;
  delta.linear = rmath::vector3d::X * desired_speed;
  delta.angular = rmath::vector3d::Z * force.angle().v();
  actuation()->governed_diff_drive()->fsm_drive(delta);

  apf2D().forces_reset();
} /* apf2D_apply() */

ckin::odometry saa_subsystemQ3D::odometry(void) const {
  return sensing()->odometry()->reading();
} /* odometry() */

double saa_subsystemQ3D::max_linear_speed(void) const {
  return actuation()->governed_diff_drive()->max_linear_speed();
} /* max_linear_speed() */

NS_END(subsystem, cosm);
