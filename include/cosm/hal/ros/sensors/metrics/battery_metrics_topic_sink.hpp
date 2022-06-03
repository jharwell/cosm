/**
 * \file battery_metrics_topic_sink.hpp
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

NS_START(cosm, hal, ros, sensors, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class battery_metrics_topic_sink
 * \ingroup hal ros sensors metrics
 *
 * \brief Sink for \ref cfsm::metrics::battery_metrics and \ref
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

NS_END(metrics, sensors, ros, hal, cosm);
