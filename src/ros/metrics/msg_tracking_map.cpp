/**
 * \file msg_tracking_map.cpp
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
} /* reset() */

NS_END(metrics, ros, cosm);
