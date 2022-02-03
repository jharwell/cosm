/**
 * \file block_transporter_metrics_topic_sink.hpp
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

#ifndef INCLUDE_COSM_ROS_FSM_METRICS_BLOCK_TRANSPORTER_METRICS_TOPIC_SINK_HPP_
#define INCLUDE_COSM_ROS_FSM_METRICS_BLOCK_TRANSPORTER_METRICS_TOPIC_SINK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/ros/metrics/topic_sink.hpp"
#include "cosm/fsm/metrics/block_transporter_metrics_data.hpp"
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
    : public cros::metrics::topic_sink<cfsm::metrics::block_transporter_metrics_data> {
 public:
  using collector_type = cfsm::metrics::block_transporter_metrics_collector;

  block_transporter_metrics_topic_sink(const std::string& topic,
                              const rmetrics::output_mode& mode,
                              const rtypes::timestep& interval)
      : topic_sink(topic, mode, interval) {}
};

NS_END(metrics, fsm, ros, cosm);

#endif /* INCLUDE_COSM_ROS_FSM_METRICS_BLOCK_TRANSPORTER_METRICS_TOPIC_SINK_HPP_ */
