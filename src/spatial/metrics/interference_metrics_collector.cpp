/**
 * \file interference_metrics_collector.cpp
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
#include "cosm/spatial/metrics/interference_metrics_collector.hpp"

#include "cosm/spatial/metrics/interference_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
interference_metrics_collector::interference_metrics_collector(
    std::unique_ptr<rmetrics::base_metrics_sink> sink)
    : base_metrics_collector(std::move(sink)) {}

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
  m_data.cum.n_entered_interference += static_cast<size_t>(m.entered_interference());

  m_data.interval.n_exited_interference +=
      static_cast<size_t>(m.exited_interference());
  m_data.cum.n_exited_interference += static_cast<size_t>(m.exited_interference());

  if (m.exited_interference()) {
    ++m_data.interval.n_episodes;
    ++m_data.cum.n_episodes;

    m_data.interval.interference_duration += m.interference_duration().v();
    m_data.cum.interference_duration += m.interference_duration().v();
  }
} /* collect() */

void interference_metrics_collector::reset_after_interval(void) {
  m_data.interval.n_episodes = 0;
  m_data.interval.n_exp_interference = 0;
  m_data.interval.n_entered_interference = 0;
  m_data.interval.n_exited_interference = 0;
  m_data.interval.interference_duration = 0;
} /* reset_after_interval() */

NS_END(metrics, spatial, cosm);
