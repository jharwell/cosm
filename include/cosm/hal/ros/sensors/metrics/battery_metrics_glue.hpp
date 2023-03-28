/**
 * \file battery_metrics_glue.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <ros/ros.h>

#include "cosm/hal/ros/sensors/metrics/battery_metrics_msg.hpp"
#include "cosm/pal/pal.hpp"
#include "cosm/ros/metrics/msg_traits.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ros::metrics::msg_traits {

template<>
struct payload_type<chros::sensors::metrics::battery_metrics_msg> {
  using type = chsensors::metrics::battery_metrics_data;
};

} /* namespace cosm::ros::metrics::msg_traits */


/*******************************************************************************
 * ROS Message Traits
 ******************************************************************************/
namespace ros {
namespace message_traits {

template<>
struct MD5Sum<chros::sensors::metrics::battery_metrics_msg> {
  static const char* value() {
    return cpal::kMsgTraitsMD5.c_str();
  }
  static const char* value(const chros::sensors::metrics::battery_metrics_msg&) {
    return value();
  }
};
template <>
struct DataType<chros::sensors::metrics::battery_metrics_msg> {
  static const char* value() {
    return "cosm_msgs/battery_metrics_data";
  }
  static const char* value(const chros::sensors::metrics::battery_metrics_msg&) {
    return value();
  }
};

template<>
struct Definition<chros::sensors::metrics::battery_metrics_msg> {
  static const char* value() {
    return "See COSM docs for documentation.";
  }
  static const char* value(const chros::sensors::metrics::battery_metrics_msg&) {
    return value();
  }
};

template<>
struct HasHeader<chros::sensors::metrics::battery_metrics_msg> : TrueType {};

} /* namespace message_traits */

namespace serialization {

template<>
struct Serializer<chros::sensors::metrics::battery_metrics_msg> {
  template<typename Stream, typename T>
  inline static void allInOne(Stream& stream, T t) {
    stream.next(t.header);

    stream.next(t.data.interval.percentage);

    /*
     * Note that we don't send the cumulative data; if we did so and it
     * accumulated on the other end we would be overcounting, and we HAVE to
     * accumulate it during collection to maintain good design.
     */
  }
  ROS_DECLARE_ALLINONE_SERIALIZER;
};

} /* namespace serialization */
} /* namespace ros */
