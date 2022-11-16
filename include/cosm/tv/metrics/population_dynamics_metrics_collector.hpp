/**
 * \file population_dynamics_metrics_collector.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <list>

#include "rcppsw/metrics/base_collector.hpp"

#include "cosm/tv/metrics/population_dynamics_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, tv, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class population_dynamics_metrics_collector
 * \ingroup metrics tv
 *
 * \brief Collector for \ref population_dynamics_metrics.
 *
 * Metrics CAN be collected in parallel; concurrent updates to the gathered
 * stats are supported.
 */
class population_dynamics_metrics_collector final : public rmetrics::base_collector {
 public:
  /**
   * \param sink The metrics sink to use.
   */
  explicit population_dynamics_metrics_collector(
      std::unique_ptr<rmetrics::base_sink> sink);

  /* base_collector overrides */
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;
  const rmetrics::base_data* data(void) const override { return &m_data; }


 private:
  /* clang-format off */
  population_dynamics_metrics_data m_data{};
};

NS_END(metrics, tv, cosm);

