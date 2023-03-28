/**
 * \file kinematics_metrics_msg.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <std_msgs/Header.h>

#include "cosm/kin/metrics/kinematics_metrics_data.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ros::spatial::metrics {

/*******************************************************************************
 * ROS Messages
 ******************************************************************************/
struct kinematics_metrics_msg {
  kinematics_metrics_msg(void) : data(0, 0) {}

  kinematics_metrics_msg(size_t n_robots, size_t n_contexts)
      : data(n_robots, n_contexts) {}

  ::std_msgs::Header               header{};
  ckin::metrics::kinematics_metrics_data data;
};

} /* namespace cosm::ros::spatial::metrics */
