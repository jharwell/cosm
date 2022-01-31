/**
 * \file distributor_metrics_collector.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
#include "cosm/foraging/block_dist/metrics/distributor_metrics_collector.hpp"

#include "cosm/foraging/block_dist/metrics/distributor_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, block_dist, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
distributor_metrics_collector::distributor_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void distributor_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = static_cast<const distributor_metrics&>(metrics);

  m_data.interval.n_configured_clusters = m.n_configured_clusters();
  m_data.interval.n_mapped_clusters = m.n_mapped_clusters();
  m_data.interval.capacity = m.capacity();
  m_data.interval.size += m.size();

  m_data.cum.n_configured_clusters = m.n_configured_clusters();
  m_data.cum.n_mapped_clusters = m.n_mapped_clusters();
  m_data.cum.capacity = m.capacity();
  m_data.cum.size += m.size();
} /* collect() */

void distributor_metrics_collector::reset_after_interval(void) {
  m_data.interval.size = 0;
} /* reset_after_interval() */

NS_END(metrics, block_dist, foraging, cosm);
