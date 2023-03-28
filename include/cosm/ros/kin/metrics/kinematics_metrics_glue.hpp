/**
 * \file kinematics_metrics_glue.hpp
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

#include "cosm/ros/kin/metrics/kinematics_metrics_msg.hpp"
#include "cosm/pal/pal.hpp"
#include "cosm/ros/metrics/msg_traits.hpp"
#include "cosm/kin/metrics/contexts.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ros::metrics::msg_traits {

template<>
struct payload_type<crsmetrics::kinematics_metrics_msg> {
  using type = ckin::metrics::kinematics_metrics_data;
};

} /* namespace cosm::ros::metrics::msg_traits */

/*******************************************************************************
 * ROS Message Traits
 ******************************************************************************/
namespace ros {
namespace message_traits {

template<>
struct MD5Sum<crsmetrics::kinematics_metrics_msg> {
  static const char* value() {
    return cpal::kMsgTraitsMD5.c_str();
  }
  static const char* value(const crsmetrics::kinematics_metrics_msg&) {
    return value();
  }
};
template <>
struct DataType<crsmetrics::kinematics_metrics_msg> {
  static const char* value() {
    return "cosm_msgs/kinematics_metrics_data";
  }
  static const char* value(const crsmetrics::kinematics_metrics_msg&) {
    return value();
  }
};

template<>
struct Definition<crsmetrics::kinematics_metrics_msg> {
  static const char* value() {
    return "See COSM docs for documentation.";
  }
  static const char* value(const crsmetrics::kinematics_metrics_msg&) {
    return value();
  }
};

template<>
struct HasHeader<crsmetrics::kinematics_metrics_msg> : TrueType {};

} /* namespace message_traits */

namespace serialization {

template<>
struct Serializer<rmath::vector3d>
{
  template<typename Stream>
  inline static void write(Stream& stream, const rmath::vector3d& t) {
    stream.next(t.x());
    stream.next(t.y());
    stream.next(t.z());
  }

  template<typename Stream>
  inline static void read(Stream& stream, rmath::vector3d& t) {
    double tmp;

    stream.next(tmp);
    t.x(tmp);

    stream.next(tmp);
    t.y(tmp);

    stream.next(tmp);
    t.z(tmp);
  }

  inline static uint32_t serializedLength(const rmath::vector3d& t) {
    uint32_t size = 0;
    size += serializationLength(t.x());
    size += serializationLength(t.y());
    size += serializationLength(t.z());
    return size;
  }
};

template<>
struct Serializer<rmath::euler_angles>
{
  template<typename Stream>
  inline static void write(Stream& stream, const rmath::euler_angles& t) {
    stream.next(t.x().v());
    stream.next(t.y().v());
    stream.next(t.z().v());
  }

  template<typename Stream>
  inline static void read(Stream& stream, rmath::euler_angles& t) {
    double x, y,z;

    stream.next(x);
    stream.next(y);
    stream.next(z);
    t = rmath::euler_angles(rmath::radians(x),
                            rmath::radians(y),
                            rmath::radians(z));
  }

  inline static uint32_t serializedLength(const rmath::euler_angles& t) {
    uint32_t size = 0;
    size += serializationLength(t.x().v());
    size += serializationLength(t.y().v());
    size += serializationLength(t.z().v());
    return size;
  }
};
template<>
struct Serializer<rspatial::euclidean_dist>
{
  template<typename Stream>
  inline static void write(Stream& stream, const rspatial::euclidean_dist& t) {
    stream.next(t.v());
  }

  template<typename Stream>
  inline static void read(Stream& stream, rspatial::euclidean_dist& t) {
    double tmp;
    stream.next(tmp);
    t.set(tmp);
  }

  inline static uint32_t serializedLength(const rspatial::euclidean_dist& t) {
    uint32_t size = 0;
    size += serializationLength(t.v());
    return size;
  }
};

template<>
struct Serializer<crsmetrics::kinematics_metrics_msg> {
  template<typename Stream, typename T>
  inline static void allInOne(Stream& stream, T t) {
    stream.next(t.header);

    for (size_t i = 0; i < ckmetrics::context_type::ekMAX; ++i) {
      for (auto &travel : t.data.interval()[i].traveled) {
        stream.next(travel);
      } /* for(&travel..) */

      for (auto &twist : t.data.interval()[i].twist) {
        stream.next(twist.linear);
        stream.next(twist.angular);
      } /* for(&twist..) */

      for (auto &pose : t.data.interval()[i].pose) {
        stream.next(pose.position);
        stream.next(pose.orientation);
      } /* for(&pose..) */
      /*
       * Note that we don't send the cumulative data; if we did so and it
       * accumulated on the other end we would be overcounting, and we HAVE to
       * accumulate it during collection to maintain good design.
       */
    } /* for(i...) */
  }
  ROS_DECLARE_ALLINONE_SERIALIZER;
};

} /* namespace serialization */
} /* namespace ros */
