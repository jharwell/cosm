/**
 * \file bi_tab_metrics_collector.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
  m_data.interval.task_depth_sw_count +=
      static_cast<size_t>(m.task_depth_changed());
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
