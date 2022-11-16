/**
 * \file convergence_metrics_collector.hpp
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

#include "cosm/convergence/metrics/convergence_metrics_data.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, convergence, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class convergence_metrics_collector
 * \ingroup convergence metrics
 *
 * \brief Collector for \ref convergence_metrics.
 *
 * Metrics are written out each timestep.
 */
class convergence_metrics_collector final : public rmetrics::base_collector {
 public:
  /**
   * \param sink The metrics sink to use.
   */
  explicit convergence_metrics_collector(
      std::unique_ptr<rmetrics::base_sink> sink);

  /* base_collector overrides */
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;
  const rmetrics::base_data* data(void) const override { return &m_data; }

 private:
  /* clang-format off */
  convergence_metrics_data m_data{};
  /* clang-format on */
};

NS_END(convergence, metrics, cosm);

