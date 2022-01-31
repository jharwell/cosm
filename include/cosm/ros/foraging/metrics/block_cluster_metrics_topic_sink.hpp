/**
 * \file block_cluster_metrics_topic_sink.hpp
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

#ifndef INCLUDE_COSM_ROS_FORAGING_METRICS_BLOCK_CLUSTER_METRICS_TOPIC_SINK_HPP_
#define INCLUDE_COSM_ROS_FORAGING_METRICS_BLOCK_CLUSTER_METRICS_TOPIC_SINK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/ros/metrics/topic_sink.hpp"
#include "cosm/foraging/metrics/block_cluster_metrics_data.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * ROS Message Traits
 ******************************************************************************/
NS_START(ros, message_traits);

template<>
struct MD5Sum<cforaging::metrics::block_cluster_metrics_data> {
  static const char* value() {
    return MD5Sum<cforaging::metrics::block_cluster_metrics_data>::value();
  }
  static const char* value(const cforaging::metrics::block_cluster_metrics_data& m) {
    return MD5Sum<cforaging::metrics::block_cluster_metrics_data>::value(m);
  }
};
template <>
struct DataType<cforaging::metrics::block_cluster_metrics_data> {
  static const char* value() {
    return DataType<cforaging::metrics::block_cluster_metrics_data>::value();
  }
  static const char* value(const cforaging::metrics::block_cluster_metrics_data& m) {
    return DataType<cforaging::metrics::block_cluster_metrics_data>::value(m);
  }
};

template<>
struct Definition<cforaging::metrics::block_cluster_metrics_data> {
  static const char* value() {
    return Definition<cforaging::metrics::block_cluster_metrics_data>::value();
  }
  static const char* value(const cforaging::metrics::block_cluster_metrics_data& m) {
    return Definition<cforaging::metrics::block_cluster_metrics_data>::value(m);
  }
};
NS_END(message_traits);

NS_START(serialization);

template<>
struct Serializer<cforaging::metrics::block_cluster_metrics_data> {
  template<typename Stream, typename T>
  inline static void allInOne(Stream& stream, T t) {
    for (auto &count : t.interval.block_counts) {
      stream.next(count.load());
    } /* for(&c..) */
    for (auto &count : t.cum.block_counts) {
      stream.next(count.load());
    } /* for(&c..) */
    for (auto &ext : t.extents) {
      stream.next(ext.area.load());
      stream.next(ext.xmin.load());
      stream.next(ext.xmax.load());
      stream.next(ext.ymin.load());
      stream.next(ext.ymax.load());
    } /* for(&c..) */
  }
  ROS_DECLARE_ALLINONE_SERIALIZER;
};

NS_END(serialization, ros);

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::foraging::metrics {
class block_cluster_metrics_collector;
} /* namespace cosm::foraging::metrics */

NS_START(cosm, ros, foraging, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_cluster_metrics_topic_sink
 * \ingroup ros foraging metrics
 *
 * \brief Sink for \ref cforaging::metrics::block_cluster_metrics and \ref
 * cforaging::metrics::block_cluster_metrics_collector to output metrics to a ROS
 * topic.
 */
class block_cluster_metrics_topic_sink final
    : public cros::metrics::topic_sink<cforaging::metrics::block_cluster_metrics_data> {
 public:
  using collector_type = cforaging::metrics::block_cluster_metrics_collector;

  block_cluster_metrics_topic_sink(const std::string& topic,
                              const rmetrics::output_mode& mode,
                              const rtypes::timestep& interval)
      : topic_sink(topic, mode, interval) {}
};

NS_END(metrics, foraging, ros, cosm);

#endif /* INCLUDE_COSM_ROS_FORAGING_METRICS_BLOCK_CLUSTER_METRICS_TOPIC_SINK_HPP_ */
