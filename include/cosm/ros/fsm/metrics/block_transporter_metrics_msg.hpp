/**
 * \file block_transporter_metrics_msg.hpp
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

#include "cosm/fsm/metrics/block_transporter_metrics_data.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ros, fsm, metrics);

/*******************************************************************************
 * ROS Messages
 ******************************************************************************/
struct block_transporter_metrics_msg {
  ::std_msgs::Header                            header{};
  cfsm::metrics::block_transporter_metrics_data data{};
};

NS_END(metrics, fsm, ros, cosm)
