/**
 * \file interference_metrics_msg.hpp
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

#include "cosm/spatial/metrics/interference_metrics_data.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ros::spatial::metrics {

/*******************************************************************************
 * ROS Messages
 ******************************************************************************/
struct interference_metrics_msg {
  ::std_msgs::Header                   header{};
  csmetrics::interference_metrics_data data{};
};

} /* namespace cosm::ros::spatial::metrics */
