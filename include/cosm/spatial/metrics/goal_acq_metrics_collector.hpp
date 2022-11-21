/**
 * \file goal_acq_metrics_collector.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <list>
#include <memory>

#include "rcppsw/metrics/base_collector.hpp"

#include "cosm/spatial/metrics/goal_acq_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::spatial::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class goal_acq_metrics_collector
 * \ingroup spatial metrics
 *
 * \brief Collector for \ref goal_acq_metrics.
 *
 * Metrics CAN be collected in parallel from robots; concurrent updates to the
 * gathered stats are supported. Metrics are written out at the end of the
 * specified interval.
 */
class goal_acq_metrics_collector final : public rmetrics::base_collector {
 public:
  /**
   * \param sink The metrics sink to use.
   */
  explicit goal_acq_metrics_collector(
      std::unique_ptr<rmetrics::base_sink> sink);

  /* base_collector overrides */
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;
  const rmetrics::base_data* data(void) const override { return &m_data; }


 private:
  /* clang-format off */
  goal_acq_metrics_data m_data{};
  /* clang-format on */
};

} /* namespace cosm::spatial::metrics */
