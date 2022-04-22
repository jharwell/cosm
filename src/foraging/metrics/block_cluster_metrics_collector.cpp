/**
 * \file block_cluster_metrics_collector.cpp
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
#include "cosm/foraging/metrics/block_cluster_metrics_collector.hpp"

#include "cosm/foraging/metrics/block_cluster_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_cluster_metrics_collector::block_cluster_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink,
    size_t n_clusters)
    : base_collector(std::move(sink)), m_data(n_clusters) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_cluster_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = static_cast<const block_cluster_metrics&>(metrics);

  m_data.interval.block_counts[m.id().v()] += m.n_blocks();
  m_data.cum.block_counts[m.id().v()] += m.n_blocks();

  ral::mt_set(m_data.extents[m.id().v()].xmin, m.ranchor2D().x());
  ral::mt_set(m_data.extents[m.id().v()].xmax,
              m.ranchor2D().x() + m.xrspan().span());
  ral::mt_set(m_data.extents[m.id().v()].ymin, m.ranchor2D().y());
  ral::mt_set(m_data.extents[m.id().v()].ymax,
              m.ranchor2D().y() + m.yrspan().span());

  ral::mt_set(m_data.extents[m.id().v()].area,
              m.xrspan().span() * m.yrspan().span());
} /* collect() */

void block_cluster_metrics_collector::reset_after_interval(void) {
  for (size_t i = 0; i < m_data.interval.block_counts.size(); ++i) {
    ral::mt_init(&m_data.interval.block_counts[i], 0U);
  } /* for(i..) */
} /* reset_after_interval() */

NS_END(metrics, foraging, cosm);
