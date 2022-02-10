/**
 * \file block_cluster_metrics_msg.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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
