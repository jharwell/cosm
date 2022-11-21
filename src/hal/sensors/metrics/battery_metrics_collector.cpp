/**
 * \file battery_metrics_collector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/sensors/metrics/battery_metrics_collector.hpp"

#include "cosm/hal/sensors/metrics/battery_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::hal::sensors::metrics {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
battery_metrics_collector::battery_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void battery_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const battery_metrics&>(metrics);

  auto percent = m.percent_remaining();

  m_data.interval.n_robots += 1;

  ral::mt_accum(m_data.interval.percentage, percent);

  m_data.cum.n_robots += 1;
  ral::mt_accum(m_data.cum.percentage, percent);
} /* collect() */

void battery_metrics_collector::reset_after_interval(void) {
  ral::mt_set(m_data.interval.percentage, 0.0);
  ral::mt_set(m_data.interval.n_robots, 0);
} /* reset_after_interval() */

} /* namespace cosm::hal::sensors::metrics */
