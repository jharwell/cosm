/**
 * \file block_transportee_metrics_topic_sink.hpp
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
#include "cosm/ros/foraging/metrics/block_transportee_metrics_glue.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::foraging::metrics {
class block_transportee_metrics_collector;
} /* namespace cosm::foraging::metrics */

NS_START(cosm, ros, foraging, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_transportee_metrics_topic_sink
 * \ingroup ros foraging metrics
 *
 * \brief Sink for \ref cforaging::metrics::block_transportee_metrics and \ref
 * cforaging::metrics::block_transportee_metrics_collector to output metrics to a ROS
 * topic.
 */
class block_transportee_metrics_topic_sink final
    : public cros::metrics::topic_sink<crfmetrics::block_transportee_metrics_msg> {
 public:
  using collector_type = cforaging::metrics::block_transportee_metrics_collector;

  block_transportee_metrics_topic_sink(const std::string& topic,
                                       const rmetrics::output_mode& mode,
                                       const rtypes::timestep& interval)
      : topic_sink(topic, mode, interval) {}
};

NS_END(metrics, foraging, ros, cosm);
