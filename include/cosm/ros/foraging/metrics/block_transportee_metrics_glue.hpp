/**
 * \file block_transportee_metrics_glue.hpp
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

#include "rcppsw/patterns/decorator/decorator.hpp"

#include "cosm/cosm.hpp"
#include "cosm/pal/pal.hpp"
#include "cosm/ros/foraging/metrics/block_transportee_metrics_msg.hpp"
#include "cosm/ros/metrics/msg_traits.hpp"
#include "cosm/foraging/metrics/block_transportee_metrics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ros::metrics::msg_traits {

template<>
struct payload_type<crfmetrics::block_transportee_metrics_msg> {
  using type = cfmetrics::block_transportee_metrics_data;
};

} /* namespace cosm::ros::metrics::msg_traits */

/*******************************************************************************
 * ROS Message Traits
 ******************************************************************************/
namespace ros::message_traits {

template<>
struct MD5Sum<crfmetrics::block_transportee_metrics_msg> {
  static const char* value() {
    return cpal::kMsgTraitsMD5.c_str();
  }
  static const char* value(const crfmetrics::block_transportee_metrics_msg& ) {
    return value();
  }
};
template <>
struct DataType<crfmetrics::block_transportee_metrics_msg> {
  static const char* value() {
    return "cosm_msgs/block_transportee_metrics_data";
  }
  static const char* value(const crfmetrics::block_transportee_metrics_msg&) {
    return value();
  }
};

template<>
struct Definition<crfmetrics::block_transportee_metrics_msg> {
  static const char* value() {
    return "See COSM docs for documentation.";
  }
  static const char* value(const crfmetrics::block_transportee_metrics_msg&) {
    return value();
  }
};
template<>
struct HasHeader<crfmetrics::block_transportee_metrics_msg> : TrueType {};

} /* namespace message_traits */

namespace serialization {

template<>
struct Serializer<crfmetrics::block_transportee_metrics_msg> {
  template<typename Stream, typename T>
  inline static void allInOne(Stream& stream, T t) {
    stream.next(t.header);

    stream.next(t.data.interval.n_transported);
    stream.next(t.data.interval.n_cube_transported);
    stream.next(t.data.interval.n_ramp_transported);
    stream.next(t.data.interval.n_transporters);
    stream.next(t.data.interval.transport_time);
    stream.next(t.data.interval.initial_wait_time);

    /*
     * Note that we don't send the cumulative data; if we did so and it
     * accumulated on the other end we would be overcounting, and we HAVE to
     * accumulate it during collection to maintain good design.
     */
  }
  ROS_DECLARE_ALLINONE_SERIALIZER;
};

} /* namespace ros::serialization */
