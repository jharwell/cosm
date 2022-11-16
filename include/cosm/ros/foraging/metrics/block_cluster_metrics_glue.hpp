/**
 * \file block_cluster_metrics_glue.hpp
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
#include <std_msgs/Header.h>

#include "cosm/cosm.hpp"
#include "cosm/ros/foraging/metrics/block_cluster_metrics_msg.hpp"
#include "cosm/pal/pal.hpp"
#include "cosm/ros/metrics/msg_traits.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ros, metrics, msg_traits);

template<>
struct payload_type<crfmetrics::block_cluster_metrics_msg> {
  using type = cfmetrics::block_cluster_metrics_data;
};

NS_END(msg_traits, metrics, ros, cosm);

/*******************************************************************************
 * ROS Message Traits
 ******************************************************************************/
NS_START(ros, message_traits);

template<>
struct MD5Sum<crfmetrics::block_cluster_metrics_msg> {
  static const char* value() {
    return cpal::kMsgTraitsMD5.c_str();
  }
  static const char* value(const crfmetrics::block_cluster_metrics_msg&) {
    return value();
  }
};
template <>
struct DataType<crfmetrics::block_cluster_metrics_msg> {
  static const char* value() {
    return "cosm_msgs/block_cluster_metrics_data";
  }
  static const char* value(const crfmetrics::block_cluster_metrics_msg&) {
    return value();
  }
};

template<>
struct Definition<crfmetrics::block_cluster_metrics_msg> {
  static const char* value() {
    return "See COSM docs for documentation.";
  }
  static const char* value(const crfmetrics::block_cluster_metrics_msg&) {
    return value();
  }
};

template<>
struct HasHeader<crfmetrics::block_cluster_metrics_msg> : TrueType {};

NS_END(message_traits);

NS_START(serialization);

template<>
struct Serializer<crfmetrics::block_cluster_metrics_msg> {
  template<typename Stream, typename T>
  inline static void allInOne(Stream& stream, T t) {
    stream.next(t.header);

    for (auto &count : t.data.interval.block_counts) {
      stream.next(count);
    } /* for(&c..) */
    /*
     * Note that we don't send the cumulative data; if we did so and it
     * accumulated on the other end we would be overcounting, and we HAVE to
     * accumulate it during collection to maintain good design.
     */
    for (auto &ext : t.data.extents) {
      stream.next(ext.area);
      stream.next(ext.xmin);
      stream.next(ext.xmax);
      stream.next(ext.ymin);
      stream.next(ext.ymax);
    } /* for(&c..) */
  }
  ROS_DECLARE_ALLINONE_SERIALIZER;
};

NS_END(serialization, ros);
