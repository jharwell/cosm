/**
 * \file movement_metrics_msg.hpp
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

#include "cosm/spatial/metrics/movement_metrics_data.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ros, spatial, metrics);

/*******************************************************************************
 * ROS Messages
 ******************************************************************************/
struct movement_metrics_msg {
  ::std_msgs::Header               header{};
  csmetrics::movement_metrics_data data{};
};

NS_END(metrics, spatial, ros, cosm)
