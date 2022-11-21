/**
 * \file movement_metrics_topic_sink.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/ros/metrics/topic_sink.hpp"
#include "cosm/ros/spatial/metrics/movement_metrics_glue.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::metrics {
class movement_metrics_collector;
} /* namespace cosm::spatial::metrics */

namespace cosm::ros::spatial::metrics {

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
    : public cros::metrics::topic_sink<crsmetrics::movement_metrics_msg> {
 public:
  using collector_type = csmetrics::movement_metrics_collector;

  movement_metrics_topic_sink(const std::string& topic,
                              const rmetrics::output_mode& mode,
                              const rtypes::timestep& interval)
      : topic_sink(topic, mode, interval) {}
};

} /* namespace cosm::ros::spatial::metrics */
