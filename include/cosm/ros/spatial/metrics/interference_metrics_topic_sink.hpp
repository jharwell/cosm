/**
 * \file interference_metrics_topic_sink.hpp
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
#include "cosm/cosm.hpp"
#include "cosm/ros/spatial/metrics/interference_metrics_glue.hpp"

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
    : public cros::metrics::topic_sink<crsmetrics::interference_metrics_msg> {
 public:
  using collector_type = csmetrics::interference_metrics_collector;

  interference_metrics_topic_sink(const std::string& topic,
                              const rmetrics::output_mode& mode,
                              const rtypes::timestep& interval)
      : topic_sink(topic, mode, interval) {}
};

NS_END(metrics, spatial, ros, cosm);
