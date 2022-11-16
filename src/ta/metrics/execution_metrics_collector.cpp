/**
 * \file execution_metrics_collector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/metrics/execution_metrics_collector.hpp"

#include "cosm/ta/metrics/execution_metrics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
execution_metrics_collector::execution_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)),
      ER_CLIENT_INIT("cosm.metrics.tasks.execution_metrics_collector") {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void execution_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const execution_metrics&>(metrics);
  ER_ASSERT(m.task_completed() || m.task_aborted(),
            "No task complete or task abort?");
  ER_ASSERT(!(m.task_completed() && m.task_aborted()),
            "Task complete AND task abort?");

  /*
   * Task finish stats are still valid because the current task in the executive
   * has not been updated (for abort or for finish).
   *
   * \todo Right now tasks update their interface/exec estimates unconditionally
   * when running, and so are the gathered metrics, but this should be able to
   * be switched on/off.
   */
  if (m.task_completed()) {
    ++m_data.interval.complete_count;
    ++m_data.cum.complete_count;
  } else if (m.task_aborted()) {
    ++m_data.interval.abort_count;
    ++m_data.cum.abort_count;
  }

  m_data.interval.interface_count += static_cast<size_t>(m.task_at_interface());
  m_data.cum.interface_count += static_cast<size_t>(m.task_at_interface());

  m_data.interval.exec_estimate +=
      static_cast<size_t>(m.task_exec_estimate().v());
  m_data.cum.exec_estimate += static_cast<size_t>(m.task_exec_estimate().v());
  m_data.interval.exec_time += static_cast<size_t>(m.task_last_exec_time().v());
  m_data.cum.exec_time += static_cast<size_t>(m.task_last_exec_time().v());

  /* Can be -1 if we aborted before getting to our task interface */
  int interface = m.task_last_active_interface();
  if (-1 != interface) {
    m_data.interval.interface_time +=
        m.task_last_interface_time(m.task_last_active_interface()).v();
    m_data.cum.interface_time +=
        m.task_last_interface_time(m.task_last_active_interface()).v();
    m_data.interval.interface_estimate += static_cast<size_t>(
        m.task_interface_estimate(m.task_last_active_interface()).v());
    m_data.cum.interface_estimate += static_cast<size_t>(
        m.task_interface_estimate(m.task_last_active_interface()).v());
  }
} /* collect() */

void execution_metrics_collector::reset_after_interval(void) {
  m_data.interval.complete_count = 0;
  m_data.interval.abort_count = 0;
  m_data.interval.interface_count = 0;
  m_data.interval.exec_time = 0;
  m_data.interval.interface_time = 0;
  m_data.interval.exec_estimate = 0;
  m_data.interval.interface_estimate = 0;
} /* reset_after_interval() */

NS_END(metrics, ta, cosm);
