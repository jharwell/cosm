/**
 * \file block_motion_metrics_collector.cpp
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
#include "cosm/foraging/metrics/block_motion_metrics_collector.hpp"

#include "cosm/foraging/metrics/block_motion_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_motion_metrics_collector::block_motion_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink) :
    base_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_motion_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = static_cast<const block_motion_metrics&>(metrics);
  m_data.interval.n_moved += m.n_moved();
  m_data.cum.n_moved += m.n_moved();
} /* collect() */

void block_motion_metrics_collector::reset_after_interval(void) {
  m_data.interval.n_moved = 0;
} /* reset_after_interval() */

NS_END(metrics, foraging, cosm);
