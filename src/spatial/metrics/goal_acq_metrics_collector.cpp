/**
 * \file goal_acq_metrics_collector.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include "cosm/spatial/metrics/goal_acq_metrics_collector.hpp"

#include "cosm/spatial/metrics/goal_acq_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
goal_acq_metrics_collector::goal_acq_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void goal_acq_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const goal_acq_metrics&>(metrics);
  auto [is_exp, true_exp] = m.is_exploring_for_goal();

  m_data.interval.n_true_exploring_for_goal +=
      static_cast<size_t>(is_exp && true_exp);
  m_data.interval.n_false_exploring_for_goal +=
      static_cast<size_t>(is_exp && !true_exp);
  m_data.interval.n_acquiring_goal +=
      static_cast<size_t>(is_exp || m.is_vectoring_to_goal());
  m_data.interval.n_vectoring_to_goal +=
      static_cast<size_t>(m.is_vectoring_to_goal());

  m_data.cum.n_true_exploring_for_goal += static_cast<size_t>(is_exp && true_exp);
  m_data.cum.n_false_exploring_for_goal +=
      static_cast<size_t>(is_exp && !true_exp);
  m_data.cum.n_acquiring_goal +=
      static_cast<size_t>(is_exp || m.is_vectoring_to_goal());
  m_data.cum.n_vectoring_to_goal += static_cast<size_t>(m.is_vectoring_to_goal());
} /* collect() */

void goal_acq_metrics_collector::reset_after_interval(void) {
  m_data.interval.n_true_exploring_for_goal = 0;
  m_data.interval.n_false_exploring_for_goal = 0;
  m_data.interval.n_acquiring_goal = 0;
  m_data.interval.n_vectoring_to_goal = 0;
} /* reset_after_interval() */

NS_END(metrics, spatial, cosm);
