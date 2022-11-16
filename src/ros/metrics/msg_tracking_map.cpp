/**
 * \file msg_tracking_map.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ros/metrics/msg_tracking_map.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ros, metrics);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void msg_tracking_map::update_on_receive(const std::string& key,
                                         int received_seq) {
  auto& tracking = decoratee().at(key);
  if (tracking.expected_seq != received_seq) {
    ER_WARN("Out of sequence packet%d for '%s': expected=%d, n_received=%zu",
            received_seq,
            key.c_str(),
            tracking.expected_seq,
            tracking.n_received);
    for (int i = tracking.expected_seq; i < received_seq; ++i) {
      tracking.missing.push_back(i);
    } /* for(i..) */
  }
  tracking.expected_seq = received_seq + 1;
  ++tracking.n_received;
} /* update_on_receive() */

void msg_tracking_map::init(const std::string& key) {
  decoratee()[key] = {};
} /* init() */

void msg_tracking_map::reset(const std::string& key) {
  auto& tracking = decoratee()[key];
  tracking.n_received = 0;
  tracking.missing = {};
  ++tracking.interval_index;
  tracking.flushed_collector = false;
  tracking.flush_ts = rtypes::constants::kNoTime;
} /* reset() */

NS_END(metrics, ros, cosm);
