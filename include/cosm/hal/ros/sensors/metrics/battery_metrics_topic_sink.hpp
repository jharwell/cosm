/**
 * \file battery_metrics_topic_sink.hpp
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
#include "cosm/hal/ros/sensors/metrics/battery_metrics_glue.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::sensors::metrics {
class battery_metrics_collector;
} /* namespace cosm::fsm::metrics */

namespace cosm::hal::ros::sensors::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class battery_metrics_topic_sink
 * \ingroup hal ros sensors metrics
 *
 * \brief Sink for \ref chsensors::metrics::battery_metrics and \ref
 * chsensors::metrics::battery_metrics_collector to output metrics to a ROS
 * topic.
 */
class battery_metrics_topic_sink final
    : public cros::metrics::topic_sink<chros::sensors::metrics::battery_metrics_msg> {
 public:
  using collector_type = chsensors::metrics::battery_metrics_collector;

  battery_metrics_topic_sink(const std::string& topic,
                             const rmetrics::output_mode& mode,
                             const rtypes::timestep& interval)
      : topic_sink(topic, mode, interval) {}
};

} /* namespace cosm::hal::ros::sensors::metrics */
