/**
 * \file nest_acq_metrics_collector.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include "cosm/spatial/strategy/metrics/nest_acq_metrics_collector.hpp"

#include "cosm/spatial/strategy/metrics/nest_acq_metrics.hpp"
#include "cosm/spatial/strategy/nest_acq/random_thresh.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, strategy, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
nest_acq_metrics_collector::nest_acq_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/

void nest_acq_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const nest_acq_metrics&>(metrics);
  auto* thresh = dynamic_cast<const cssnest_acq::random_thresh*>(m.nest_acq_strategy());
  if (nullptr == thresh) {
    return;
  }

  if (auto current = thresh->thresh()) {
    ++m_data.interval.n_random_thresh;
    ++m_data.cum.n_random_thresh;

    ral::mt_add(m_data.interval.random_thresh, current->v());
    ral::mt_add(m_data.cum.random_thresh, current->v());
  }
} /* collect() */

void nest_acq_metrics_collector::reset_after_interval(void) {
  m_data.interval.random_thresh = 0.0;
  m_data.interval.n_random_thresh = 0;
} /* reset_after_interval() */

NS_END(metrics, strategy, spatial, cosm);
