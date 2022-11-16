/**
 * \file block_cluster_metrics_msg.hpp
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

#include "cosm/foraging/metrics/block_cluster_metrics_data.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ros, foraging, metrics);

/*******************************************************************************
 * ROS Messages
 ******************************************************************************/
struct block_cluster_metrics_msg {
  block_cluster_metrics_msg(void) : data(0) {}

  explicit block_cluster_metrics_msg(size_t n_clusters): data(n_clusters) {}

  ::std_msgs::Header                    header{};
  cfmetrics::block_cluster_metrics_data data;
};

NS_END(metrics, foraging, ros, cosm)
