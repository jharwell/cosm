/**
 * \file distributor_metrics_collector.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/metrics/base_collector.hpp"

#include "cosm/foraging/block_dist/metrics/distributor_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::block_dist::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class distributor_metrics_collector
 * \ingroup foraging block_dist metrics
 *
 * \brief Collector for \ref distributor_metrics.
 */
class distributor_metrics_collector final : public rmetrics::base_collector {
 public:
  /**
  * \param sink The metrics sink to use.
  */
  explicit distributor_metrics_collector(
      std::unique_ptr<rmetrics::base_sink> sink);

  /* base_collector overrides */
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;
  const rmetrics::base_data* data(void) const override { return &m_data; }

 private:
  /* clang-format off */
  distributor_metrics_data m_data{};
  /* clang-format on */
};

} /* namespace cosm::foraging::block_dist::metrics */

