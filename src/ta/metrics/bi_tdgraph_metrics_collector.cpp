/**
 * \file bi_tdgraph_metrics_collector.cpp
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
#include "cosm/ta/metrics/bi_tdgraph_metrics_collector.hpp"

#include <cmath>

#include "cosm/ta/metrics/bi_tdgraph_metrics.hpp"
#include "cosm/ta/polled_task.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
bi_tdgraph_metrics_collector::bi_tdgraph_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink,
    size_t decomposition_depth)
    : base_collector(std::move(sink)),
      m_data(decomposition_depth) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void bi_tdgraph_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const bi_tdgraph_metrics&>(metrics);
  ++m_data.interval.depth_counts[m.current_task_depth()];
  ++m_data.interval.task_counts[m.current_task_id()];
  ++m_data.interval.tab_counts[m.current_task_tab()];

  ++m_data.cum.depth_counts[m.current_task_depth()];
  ++m_data.cum.task_counts[m.current_task_id()];
  ++m_data.cum.tab_counts[m.current_task_tab()];
} /* collect() */

void bi_tdgraph_metrics_collector::reset_after_interval(void) {
  for (size_t i = 0; i < m_data.interval.depth_counts.size(); ++i) {
    ral::mt_init(&m_data.interval.depth_counts[i], 0U);
  } /* for(i..) */

  for (size_t i = 0; i < m_data.interval.task_counts.size(); ++i) {
    ral::mt_init(&m_data.interval.task_counts[i], 0U);
  } /* for(i..) */

  for (size_t i = 0; i < m_data.interval.tab_counts.size(); ++i) {
    ral::mt_init(&m_data.interval.tab_counts[i], 0U);
  } /* for(i..) */
} /* reset_after_interval() */

NS_END(metrics, ta, cosm);
