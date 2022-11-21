/**
 * \file bi_tdgraph_metrics_collector.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/metrics/base_collector.hpp"
#include "rcppsw/rcppsw.hpp"

#include "cosm/ta/metrics/bi_tdgraph_metrics_data.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class bi_tdgraph_metrics_collector
 * \ingroup ta metrics
 *
 * \brief Collector for metrics about the current task_dist of tasks in a
 * collection of agents executing tasks in a \ref cta::ds::bi_tdgraph.
 *
 * Metrics CAN be collected in parallel from robots; concurrent updates to the
 * gathered stats are supported. Metrics are written out at the specified
 * interval.
 */
class bi_tdgraph_metrics_collector final : public rmetrics::base_collector {
 public:
  /**
   * \param sink The metrics sink to use.
   * \param decomposition_depth The maximum depth of the \ref cta::ds::bi_tdgraph.
   */
  explicit bi_tdgraph_metrics_collector(
      std::unique_ptr<rmetrics::base_sink> sink,
      size_t decomposition_depth);

  /* base_collector overrides */
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;
  const rmetrics::base_data* data(void) const override { return &m_data; }

  /* const std::vector<std::atomic_size_t>& int_task_counts(void) const { */
  /*   return m_int_task_counts; */
  /* } */

 private:
  /* clang-format off */
  bi_tdgraph_metrics_data m_data;
};

} /* namespace cosm::ta::metrics */
