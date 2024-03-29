/**
 * \file battery_metrics_msg.hpp
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

#include "cosm/hal/sensors/metrics/battery_metrics_data.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::ros::sensors::metrics {

/*******************************************************************************
 * ROS Messages
 ******************************************************************************/
struct battery_metrics_msg {
  ::std_msgs::Header                       header{};
  chsensors::metrics::battery_metrics_data data{};
};

} /* namespace cosm::ros::sensors::metrics */
