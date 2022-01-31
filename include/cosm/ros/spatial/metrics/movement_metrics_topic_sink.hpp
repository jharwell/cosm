/**
 * \file movement_metrics_topic_sink.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_ROS_METRICS_SPATIAL_MOVEMENT_METRICS_TOPIC_SINK_HPP_
#define INCLUDE_COSM_ROS_METRICS_SPATIAL_MOVEMENT_METRICS_TOPIC_SINK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/ros/metrics/topic_sink.hpp"
#include "cosm/spatial/metrics/movement_metrics_data.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * ROS Message Traits
 ******************************************************************************/
NS_START(ros, message_traits);

template<>
struct MD5Sum<csmetrics::movement_metrics_data> {
  static const char* value() {
    return MD5Sum<csmetrics::movement_metrics_data>::value();
  }
  static const char* value(const csmetrics::movement_metrics_data& m) {
    return MD5Sum<csmetrics::movement_metrics_data>::value(m);
  }
};
template <>
struct DataType<csmetrics::movement_metrics_data> {
  static const char* value() {
    return DataType<csmetrics::movement_metrics_data>::value();
  }
  static const char* value(const csmetrics::movement_metrics_data& m) {
    return DataType<csmetrics::movement_metrics_data>::value(m);
  }
};

template<>
struct Definition<csmetrics::movement_metrics_data> {
  static const char* value() {
    return Definition<csmetrics::movement_metrics_data>::value();
  }
  static const char* value(const csmetrics::movement_metrics_data& m) {
    return Definition<csmetrics::movement_metrics_data>::value(m);
  }
};
NS_END(message_traits);

NS_START(serialization);

template<>
struct Serializer<csmetrics::movement_metrics_data> {
  template<typename Stream, typename T>
  inline static void allInOne(Stream& stream, T t) {
    for (auto &m : t.interval) {
      stream.next(m.distance.load());
      stream.next(m.n_robots.load());
      stream.next(m.velocity.load());
    } /* for(&m..) */
    for (auto &m : t.cum) {
      stream.next(m.distance.load());
      stream.next(m.n_robots.load());
      stream.next(m.velocity.load());
    } /* for(&m..) */
  }
  ROS_DECLARE_ALLINONE_SERIALIZER;
};

NS_END(serialization, ros);

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::metrics {
class movement_metrics_collector;
} /* namespace cosm::spatial::metrics */

NS_START(cosm, ros, spatial, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class movement_metrics_topic_sink
 * \ingroup ros spatial metrics
 *
 * \brief Sink for \ref csmetrics::movement_metrics and \ref
 * csmetrics::movement_metrics_collector to output metrics to a ROS topic.
 */
class movement_metrics_topic_sink final
    : public cros::metrics::topic_sink<csmetrics::movement_metrics_data> {
 public:
  using collector_type = csmetrics::movement_metrics_collector;

  movement_metrics_topic_sink(const std::string& topic,
                              const rmetrics::output_mode& mode,
                              const rtypes::timestep& interval)
      : topic_sink(topic, mode, interval) {}
};

NS_END(metrics, spatial, ros, cosm);

#endif /* INCLUDE_COSM_ROS_METRICS_SPATIAL_MOVEMENT_METRICS_TOPIC_SINK_HPP_ */
