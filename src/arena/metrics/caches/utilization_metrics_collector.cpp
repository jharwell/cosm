/**
 * \file utilization_metrics_collector.cpp
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
#include "cosm/arena/metrics/caches/utilization_metrics_collector.hpp"

#include "cosm/arena/metrics/caches/utilization_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, metrics, caches);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
utilization_metrics_collector::utilization_metrics_collector(
    std::unique_ptr<rmetrics::base_metrics_sink> sink)
    : base_metrics_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void utilization_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const utilization_metrics&>(metrics);

  m_data.interval.n_pickups += m.total_block_pickups();
  m_data.interval.n_drops += m.total_block_drops();
  m_data.interval.n_blocks += m.n_blocks();

  m_data.cum.n_pickups += m.total_block_pickups();
  m_data.cum.n_drops += m.total_block_drops();
  m_data.cum.n_blocks += m.n_blocks();

  ++m_data.interval.cache_count;
  ++m_data.cum.cache_count;
} /* collect() */

void utilization_metrics_collector::reset_after_interval(void) {
  m_data.interval.n_blocks = 0;
  m_data.interval.n_pickups = 0;
  m_data.interval.n_drops = 0;
  m_data.interval.cache_count = 0;
} /* resedt_after_interval() */

NS_END(caches, metrics, arena, cosm);
