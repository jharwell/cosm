/**
 * \file kinematics_metrics_topic_sink.hpp
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
#include "cosm/ros/kin/metrics/kinematics_metrics_glue.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::kin::metrics {
class kinematics_metrics_collector;
} /* namespace cosm::spatial::metrics */

namespace cosm::ros::kin::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class kinematics_metrics_topic_sink
 * \ingroup ros spatial metrics
 *
 * \brief Sink for \ref csmetrics::kinematics_metrics and \ref
 * csmetrics::kinematics_metrics_collector to output metrics to a ROS topic.
 */
class kinematics_metrics_topic_sink final
    : public cros::metrics::topic_sink<crsmetrics::kinematics_metrics_msg> {
 public:
  using collector_type = ckmetrics::kinematics_metrics_collector;

  kinematics_metrics_topic_sink(const std::string& topic,
                              const rmetrics::output_mode& mode,
                              const rtypes::timestep& interval)
      : topic_sink(topic, mode, interval) {}
};

} /* namespace cosm::ros::kin::metrics */
