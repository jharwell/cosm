/**
 * \file nest_zone_metrics_collector.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/metrics/nest_zone_metrics_collector.hpp"

#include "cosm/spatial/metrics/nest_zone_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::spatial::metrics {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
nest_zone_metrics_collector::nest_zone_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/

void nest_zone_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const nest_zone_metrics&>(metrics);

  /* metrics common to all zoneuisition strategies */
  m_data.interval.n_in_nest += m.in_nest();
  m_data.interval.n_entered_nest += m.entered_nest();
  m_data.interval.n_exited_nest += m.exited_nest();
  m_data.interval.nest_duration += m.nest_duration().v();

  if (m.in_nest() && m_data.cum.n_in_nest == 0) {
    m_data.cum.first_nest_entry_time = m.nest_entry_time().v();
  }
  m_data.cum.n_in_nest += m.in_nest();
  m_data.cum.n_entered_nest += m.entered_nest();
  m_data.cum.n_exited_nest += m.exited_nest();
  m_data.cum.nest_duration += m.nest_duration().v();
} /* collect() */

void nest_zone_metrics_collector::reset_after_interval(void) {
  m_data.interval.n_in_nest = 0;
  m_data.interval.n_entered_nest = 0;
  m_data.interval.n_exited_nest = 0;
  m_data.interval.nest_duration = 0;
} /* reset_after_interval() */

} /* namespace cosm::spatial::metrics */
