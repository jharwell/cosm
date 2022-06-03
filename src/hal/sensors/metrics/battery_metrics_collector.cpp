/**
 * \file battery_metrics_collector.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/sensors/metrics/battery_metrics_collector.hpp"

#include "cosm/hal/sensors/metrics/battery_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, sensors, metrics);

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

NS_END(metrics, sensors, hal, cosm);
