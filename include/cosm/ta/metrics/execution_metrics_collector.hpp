/**
 * \file execution_metrics_collector.hpp
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
#include "rcppsw/er/client.hpp"

#include "cosm/ta/metrics/execution_metrics_data.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class execution_metrics_collector
 * \ingroup ta metrics
 *
 * \brief Collector for metrics for an executable task across executions of that
 * task.
 *
 * Metrics CAN be collected in parallel from robots; concurrent updates to the
 * gathered stats are supported. Metrics are output at the specified interval
 */
class execution_metrics_collector final : public rmetrics::base_collector,
                                          public rer::client<execution_metrics_collector> {
 public:
  /**
   * \param sink The metrics sink to use.
   */
  explicit execution_metrics_collector(
      std::unique_ptr<rmetrics::base_sink> sink);

  /* base_collector overrides */
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;
  const rmetrics::base_data* data(void) const override { return &m_data; }

 private:
  /* clang-format off */
  execution_metrics_data m_data{};
  /* clang-format on */
};

} /* namespace cosm::ta::metrics */

