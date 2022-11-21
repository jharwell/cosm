/**
 * \file movement_metrics_glue.hpp
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

#include "cosm/ros/spatial/metrics/movement_metrics_msg.hpp"
#include "cosm/pal/pal.hpp"
#include "cosm/ros/metrics/msg_traits.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ros::metrics::msg_traits {

template<>
struct payload_type<crsmetrics::movement_metrics_msg> {
  using type = csmetrics::movement_metrics_data;
};

} /* namespace cosm::ros::metrics::msg_traits */

/*******************************************************************************
 * ROS Message Traits
 ******************************************************************************/
namespace ros::message_traits {

template<>
struct MD5Sum<crsmetrics::movement_metrics_msg> {
  static const char* value() {
    return cpal::kMsgTraitsMD5.c_str();
  }
  static const char* value(const crsmetrics::movement_metrics_msg&) {
    return value();
  }
};
template <>
struct DataType<crsmetrics::movement_metrics_msg> {
  static const char* value() {
    return "cosm_msgs/movement_metrics_data";
  }
  static const char* value(const crsmetrics::movement_metrics_msg&) {
    return value();
  }
};

template<>
struct Definition<crsmetrics::movement_metrics_msg> {
  static const char* value() {
    return "See COSM docs for documentation.";
  }
  static const char* value(const crsmetrics::movement_metrics_msg&) {
    return value();
  }
};

template<>
struct HasHeader<crsmetrics::movement_metrics_msg> : TrueType {};

} /* namespace message_traits */

namespace serialization {

template<>
struct Serializer<crsmetrics::movement_metrics_msg> {
  template<typename Stream, typename T>
  inline static void allInOne(Stream& stream, T t) {
    stream.next(t.header);

    for (size_t i = 0; i < csmetrics::movement_category::ekMAX; ++i) {
      stream.next(t.data.interval[i].distance);
      stream.next(t.data.interval[i].n_robots);
      stream.next(t.data.interval[i].velocity);

      /*
       * Note that we don't send the cumulative data; if we did so and it
       * accumulated on the other end we would be overcounting, and we HAVE to
       * accumulate it during collection to maintain good design.
       */
    } /* for(&m..) */
  }
  ROS_DECLARE_ALLINONE_SERIALIZER;
};

} /* namespace ros::serialization */
