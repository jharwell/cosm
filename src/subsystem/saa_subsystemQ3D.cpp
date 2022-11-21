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

#include "cosm/subsystem/actuation_subsystem.hpp"
#include "cosm/subsystem/sensing_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
saa_subsystemQ3D::saa_subsystemQ3D(sensor_map&& sensors,
                                   actuator_map&& actuators,
                                   const apf2D::config::apf_manager_config* const apf_config)
    : ER_CLIENT_INIT("cosm.subsystem.saa_subsystemQ3D"),
      base_saa_subsystem(std::move(sensors), std::move(actuators), apf_config) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void saa_subsystemQ3D::apf_apply(void) {
  if (!apf().is_enabled()) {
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
           rcppsw::to_string(apf().value()).c_str(),
           rcppsw::to_string(apf().value().angle()).c_str(),
           rcppsw::to_string(apf().value().length()).c_str());
  RCPPSW_UNUSED double applied =
      actuation()->governed_diff_drive()->applied_throttle();
  double active = actuation()->governed_diff_drive()->active_throttle();
  ER_DEBUG("Applied throttle: %f active throttle: %f", applied, active);

  auto force = apf().value();
  auto desired_speed = force.length() * (1.0 - active);
  ckin::twist delta;
  delta.linear = rmath::vector3d::X * desired_speed;
  delta.angular = rmath::vector3d::Z * force.angle().v();
  actuation()->governed_diff_drive()->fsm_drive(delta);

  apf().forces_reset();
} /* apf_apply() */

} /* namespace cosm::subsystem */
