/**
 * \file interference_metrics_collector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/metrics/interference_metrics_collector.hpp"

#include "cosm/spatial/metrics/interference_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::spatial::metrics {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
interference_metrics_collector::interference_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void interference_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const interference_metrics&>(metrics);

  m_data.interval.n_exp_interference += static_cast<size_t>(m.exp_interference());
  m_data.cum.n_exp_interference += static_cast<size_t>(m.exp_interference());

  m_data.interval.n_entered_interference +=
      static_cast<size_t>(m.entered_interference());
  m_data.cum.n_entered_interference +=
      static_cast<size_t>(m.entered_interference());

  m_data.interval.n_exited_interference +=
      static_cast<size_t>(m.exited_interference());
  m_data.cum.n_exited_interference +=
      static_cast<size_t>(m.exited_interference());

  if (m.exited_interference()) {
    ++m_data.interval.n_episodes;
    ++m_data.cum.n_episodes;

    m_data.interval.interference_duration += m.interference_duration()->v();
    m_data.cum.interference_duration += m.interference_duration()->v();
  }
} /* collect() */

void interference_metrics_collector::reset_after_interval(void) {
  m_data.interval.n_episodes = 0;
  m_data.interval.n_exp_interference = 0;
  m_data.interval.n_entered_interference = 0;
  m_data.interval.n_exited_interference = 0;
  m_data.interval.interference_duration = 0;
} /* reset_after_interval() */

} /* namespace cosm::spatial::metrics */
