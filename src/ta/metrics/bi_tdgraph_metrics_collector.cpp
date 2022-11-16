/**
 * \file bi_tdgraph_metrics_collector.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
    : base_collector(std::move(sink)), m_data(decomposition_depth) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void bi_tdgraph_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const bi_tdgraph_metrics&>(metrics);

  auto depth = m.current_task_depth();
  auto id = m.current_task_id();
  auto tab = m.current_task_tab();

  if (-1 != depth) {
    ++m_data.interval.depth_counts[m.current_task_depth()];
    ++m_data.cum.depth_counts[m.current_task_depth()];
  }

  if (-1 != id) {
    ++m_data.interval.task_counts[m.current_task_id()];
    ++m_data.cum.task_counts[m.current_task_id()];
  }

  if (-1 != tab) {
    ++m_data.interval.tab_counts[m.current_task_tab()];
    ++m_data.cum.tab_counts[m.current_task_tab()];
  }
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
