/**
 * \file bi_tab_metrics_collector.hpp
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

#include "cosm/ta/metrics/bi_tab_metrics_data.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class bi_tab_metrics_collector
 * \ingroup ta metrics
 *
 * \brief Collector for metrics for \ref bi_tab_metrics about task allocation.
 *
 * Metrics CAN be collected in parallel from robots; concurrent updates to the
 * gathered stats are supported. Metrics should only be collected upon
 * completion/abortion of a task. Metrics are written out at the specified
 * interval.
 */
class bi_tab_metrics_collector final : public rmetrics::base_collector {
 public:
  /**
   * \param sink The metrics sink to use.
   */
  explicit bi_tab_metrics_collector(
      std::unique_ptr<rmetrics::base_sink> sink);

  /* base_collector overrides */
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;
  const rmetrics::base_data* data(void) const override { return &m_data; }


 private:
  /* clang-format off */
  bi_tab_metrics_data m_data{};
};

} /* namespace cosm::ta::metrics */
