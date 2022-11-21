/**
 * \file block_transportee_metrics_msg.hpp
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

#include "cosm/foraging/metrics/block_transportee_metrics_data.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ros::foraging::metrics {

/*******************************************************************************
 * ROS Messages
 ******************************************************************************/
struct block_transportee_metrics_msg {
  ::std_msgs::Header                        header{};
  cfmetrics::block_transportee_metrics_data data{};
};

}
