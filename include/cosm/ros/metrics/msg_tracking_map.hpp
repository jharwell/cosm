/**
 * \file msg_tracking_map.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
