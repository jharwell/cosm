/**
 * \file msg_tracking_map.hpp
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
#include <map>
#include <vector>
#include <string>

#include "rcppsw/patterns/decorator/decorator.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ros, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
struct msg_tracking_map_value {
  std::vector<int> missing{};
  size_t n_received{};
  int expected_seq{0};
  size_t interval_index{0};

  bool flushed_collector{false};
  rtypes::timestep flush_ts{rtypes::constants::kNoTime};
};

/**
 * \class msg_tracking_map
 * \ingroup ros metrics
 *
 * \brief Maps a collector name to the (# of metric packets received, list of
 * missing packets) pair.
 */
class msg_tracking_map
    : public rer::client<msg_tracking_map>,
      private rpdecorator::decorator<std::map<std::string,
                                              msg_tracking_map_value>
                                     > {
 public:
  msg_tracking_map(void)
      : ER_CLIENT_INIT("cosm.ros.metrics.msg_tracking_map") {}

  RCPPSW_DECORATE_DECLDEF(operator[]);

  void update_on_receive(const std::string& key, int recived_seq);
  void init(const std::string& key);
  void reset(const std::string& key);
};

NS_END(metrics, ros, cosm);
