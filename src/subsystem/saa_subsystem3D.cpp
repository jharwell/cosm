/**
 * \file saa_subsystem3D.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/subsystem/saa_subsystem3D.hpp"

#include "cosm/subsystem/actuation_subsystem.hpp"
#include "cosm/subsystem/sensing_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
saa_subsystem3D::saa_subsystem3D(sensor_map&& sensors,
                                 actuator_map&& actuators,
                                 const apf2D::config::apf_manager_config* const apf_config)
    : ER_CLIENT_INIT("cosm.subsystem.saa_subsystem3D"),
      base_saa_subsystem(std::move(sensors), std::move(actuators), apf_config) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void saa_subsystem3D::apf_apply(void) {
  if (!apf().is_enabled()) {
    ER_DEBUG("Skipping applying potential fields--disabled");
    return;
  }
  auto odom = odometry();
  ER_DEBUG("odom.position=%s azimuth=%s zenith=%s",
           rcppsw::to_string(odom.pose.position).c_str(),
           rcppsw::to_string(odom.pose.orientation.z()).c_str(),
           rcppsw::to_string(odom.pose.orientation.y()).c_str());
  ER_DEBUG("odom.twist.linear=%s@(%s,%s,%s) [%s]",
           rcppsw::to_string(odom.twist.linear).c_str(),
           rcppsw::to_string(odom.twist.linear.xangle()).c_str(),
           rcppsw::to_string(odom.twist.linear.yangle()).c_str(),
           rcppsw::to_string(odom.twist.linear.zangle()).c_str(),
           rcppsw::to_string(odom.twist.linear.length()).c_str());
  ER_DEBUG("odom.twist.angular=%s [%s]",
           rcppsw::to_string(odom.twist.angular).c_str(),
           rcppsw::to_string(odom.twist.angular.length()).c_str());

  ER_DEBUG("Net APF=%s@%s [%s]",
           rcppsw::to_string(apf().value()).c_str(),
           rcppsw::to_string(apf().value().angle()).c_str(),
           rcppsw::to_string(apf().value().length()).c_str());

  auto force = apf().value();
  ckin::pose desired = {};

  /**
   * \todo This is correct ONLY for position based quadrotor locomotion
   * actuators where the command position is always interpreted relative to its
   * initial position.
   */
  desired.position = odom.pose.position + rmath::vector3d(force);

  actuation()->locomotion()->set_from_pose(desired);
  apf().forces_reset();
} /* apf_apply() */

} /* namespace cosm::subsystem */
