/**
 * \file bi_tab_metrics_collector.cpp
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
#include "cosm/ta/metrics/bi_tab_metrics_collector.hpp"

#include "cosm/ta/metrics/bi_tab_metrics.hpp"
#include "cosm/ta/polled_task.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
bi_tab_metrics_collector::bi_tab_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void bi_tab_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const bi_tab_metrics&>(metrics);
  if (m.employed_partitioning()) {
    ++m_data.interval.partition_count;
    ++m_data.cum.partition_count;
    m_data.interval.subtask1_count += static_cast<size_t>(m.subtask1_active());
    m_data.cum.subtask1_count += static_cast<size_t>(m.subtask1_active());
    m_data.interval.subtask2_count += static_cast<size_t>(m.subtask2_active());
    m_data.cum.subtask2_count += static_cast<size_t>(m.subtask2_active());

    ral::mt_accum(m_data.interval.partition_prob, m.partition_prob());
    ral::mt_accum(m_data.interval.subtask_sel_prob, m.subtask_selection_prob());

    ral::mt_accum(m_data.cum.partition_prob, m.partition_prob());
    ral::mt_accum(m_data.cum.subtask_sel_prob, m.subtask_selection_prob());
  } else {
    ++m_data.interval.no_partition_count;
    ++m_data.cum.no_partition_count;
  }

  m_data.interval.task_sw_count += static_cast<size_t>(m.task_changed());
  m_data.cum.task_sw_count += static_cast<size_t>(m.task_changed());
  m_data.interval.task_depth_sw_count += static_cast<size_t>(m.task_depth_changed());
  m_data.cum.task_depth_sw_count += static_cast<size_t>(m.task_depth_changed());
} /* collect() */

void bi_tab_metrics_collector::reset_after_interval(void) {
  m_data.interval.subtask1_count = 0;
  m_data.interval.subtask2_count = 0;
  m_data.interval.partition_count = 0;
  m_data.interval.no_partition_count = 0;
  m_data.interval.task_sw_count = 0;
  m_data.interval.task_depth_sw_count = 0;
  m_data.interval.partition_prob = 0.0;
  m_data.interval.subtask_sel_prob = 0.0;
} /* reset_after_interval() */

NS_END(metrics, ta, cosm);
