/**
 * \file block_transporter_metrics_topic_sink.hpp
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
#include "cosm/ros/fsm/metrics/block_transporter_metrics_glue.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::fsm::metrics {
class block_transporter_metrics_collector;
} /* namespace cosm::fsm::metrics */

NS_START(cosm, ros, fsm, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_transporter_metrics_topic_sink
 * \ingroup ros fsm metrics
 *
 * \brief Sink for \ref cfsm::metrics::block_transporter_metrics and \ref
 * cfsm::metrics::block_transporter_metrics_collector to output metrics to a ROS
 * topic.
 */
class block_transporter_metrics_topic_sink final
    : public cros::metrics::topic_sink<crfsm::metrics::block_transporter_metrics_msg> {
 public:
  using collector_type = cfsm::metrics::block_transporter_metrics_collector;

  block_transporter_metrics_topic_sink(const std::string& topic,
                              const rmetrics::output_mode& mode,
                              const rtypes::timestep& interval)
      : topic_sink(topic, mode, interval) {}
};

NS_END(metrics, fsm, ros, cosm);
