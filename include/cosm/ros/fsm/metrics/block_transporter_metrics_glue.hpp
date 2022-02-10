/**
 * \file block_transporter_metrics_glue.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
p * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <ros/ros.h>

#include "cosm/ros/fsm/metrics/block_transporter_metrics_msg.hpp"
#include "cosm/pal/pal.hpp"
#include "cosm/ros/metrics/msg_traits.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ros, metrics, msg_traits);

template<>
struct payload_type<crfsm::metrics::block_transporter_metrics_msg> {
  using type = cfsm::metrics::block_transporter_metrics_data;
};

NS_END(msg_traits, metrics, ros, cosm);


/*******************************************************************************
 * ROS Message Traits
 ******************************************************************************/
NS_START(ros, message_traits);

template<>
struct MD5Sum<crfsm::metrics::block_transporter_metrics_msg> {
  static const char* value() {
    return cpal::kMsgTraitsMD5.c_str();
  }
  static const char* value(const crfsm::metrics::block_transporter_metrics_msg&) {
    return value();
  }
};
template <>
struct DataType<crfsm::metrics::block_transporter_metrics_msg> {
  static const char* value() {
    return "cosm_msgs/block_transporter_metrics_data";
  }
  static const char* value(const crfsm::metrics::block_transporter_metrics_msg&) {
    return value();
  }
};

template<>
struct Definition<crfsm::metrics::block_transporter_metrics_msg> {
  static const char* value() {
    return "See COSM docs for documentation.";
  }
  static const char* value(const crfsm::metrics::block_transporter_metrics_msg&) {
    return value();
  }
};

template<>
struct HasHeader<crfsm::metrics::block_transporter_metrics_msg> : TrueType {};

NS_END(message_traits);

NS_START(serialization);

template<>
struct Serializer<crfsm::metrics::block_transporter_metrics_msg> {
  template<typename Stream, typename T>
  inline static void allInOne(Stream& stream, T t) {
    stream.next(t.header);

    stream.next(t.data.interval.n_phototaxiing_to_goal_including_ca);
    stream.next(t.data.interval.n_phototaxiing_to_goal_no_ca);

    stream.next(t.data.cum.n_phototaxiing_to_goal_including_ca);
    stream.next(t.data.cum.n_phototaxiing_to_goal_no_ca);
  }
  ROS_DECLARE_ALLINONE_SERIALIZER;
};

NS_END(serialization, ros);
