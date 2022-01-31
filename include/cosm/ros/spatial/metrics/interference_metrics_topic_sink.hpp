/**
 * \file interference_metrics_topic_sink.hpp
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

#ifndef INCLUDE_COSM_ROS_SPATIAL_METRICS_INTERFERENCE_METRICS_TOPIC_SINK_HPP_
#define INCLUDE_COSM_ROS_SPATIAL_METRICS_INTERFERENCE_METRICS_TOPIC_SINK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/ros/metrics/topic_sink.hpp"
#include "cosm/cosm.hpp"
#include "cosm/spatial/metrics/interference_metrics_data.hpp"

/*******************************************************************************
 * ROS Message Traits
 ******************************************************************************/
NS_START(ros, message_traits);

template<>
struct MD5Sum<csmetrics::interference_metrics_data> {
  static const char* value() {
    return MD5Sum<csmetrics::interference_metrics_data>::value();
  }
  static const char* value(const csmetrics::interference_metrics_data& m) {
    return MD5Sum<csmetrics::interference_metrics_data>::value(m);
  }
};
template <>
struct DataType<csmetrics::interference_metrics_data> {
  static const char* value() {
    return DataType<csmetrics::interference_metrics_data>::value();
  }
  static const char* value(const csmetrics::interference_metrics_data& m) {
    return DataType<csmetrics::interference_metrics_data>::value(m);
  }
};

template<>
struct Definition<csmetrics::interference_metrics_data> {
  static const char* value() {
    return Definition<csmetrics::interference_metrics_data>::value();
  }
  static const char* value(const csmetrics::interference_metrics_data& m) {
    return Definition<csmetrics::interference_metrics_data>::value(m);
  }
};
NS_END(message_traits);

NS_START(serialization);

template<>
struct Serializer<csmetrics::interference_metrics_data> {
  template<typename Stream, typename T>
  inline static void allInOne(Stream& stream, T t) {
    stream.next(t.interval.n_exp_interference.load());
    stream.next(t.interval.n_episodes.load());
    stream.next(t.interval.n_entered_interference.load());
    stream.next(t.interval.n_exited_interference.load());
    stream.next(t.interval.interference_duration.load());

    stream.next(t.cum.n_exp_interference.load());
    stream.next(t.cum.n_episodes.load());
    stream.next(t.cum.n_entered_interference.load());
    stream.next(t.cum.n_exited_interference.load());
    stream.next(t.cum.interference_duration.load());
  }
  ROS_DECLARE_ALLINONE_SERIALIZER;
};

NS_END(serialization, ros);

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::metrics {
class interference_metrics_collector;
} /* namespace cosm::spatial::metrics */

NS_START(cosm, ros, spatial, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class interference_metrics_topic_sink
 * \ingroup ros spatial metrics
 *
 * \brief Sink for \ref csmetrics::interference_metrics and \ref
 * csmetrics::interference_metrics_collector to output metrics to a ROS topic.
 */
class interference_metrics_topic_sink final
    : public cros::metrics::topic_sink<csmetrics::interference_metrics_data> {
 public:
  using collector_type = csmetrics::interference_metrics_collector;

  interference_metrics_topic_sink(const std::string& topic,
                              const rmetrics::output_mode& mode,
                              const rtypes::timestep& interval)
      : topic_sink(topic, mode, interval) {}
};

NS_END(metrics, spatial, ros, cosm);

#endif /* INCLUDE_COSM_ROS_SPATIAL_METRICS_INTERFERENCE_METRICS_TOPIC_SINK_HPP_ */
