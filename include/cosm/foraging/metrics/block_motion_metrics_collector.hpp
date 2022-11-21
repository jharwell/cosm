/**
 * \file block_motion_metrics_collector.hpp
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
#include <string>
#include <list>

#include "rcppsw/metrics/base_collector.hpp"

#include "cosm/cosm.hpp"
#include "cosm/foraging/metrics/block_motion_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_motion_metrics_collector
 * \ingroup foraging metrics
 *
 * \brief Collector for \ref block_motion_metrics.
 */
class block_motion_metrics_collector final : public rmetrics::base_collector {
 public:
  /**
   * \param sink The metrics sink to use.
   */
  block_motion_metrics_collector(
      std::unique_ptr<rmetrics::base_sink> sink);

  /* base_collector overrides */
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;
  const rmetrics::base_data* data(void) const override { return &m_data; }

 private:
  /* clang-format off */
  block_motion_metrics_data m_data{};
  /* clang-format on */
};

} /* namespace cosm::foraging::metrics */

