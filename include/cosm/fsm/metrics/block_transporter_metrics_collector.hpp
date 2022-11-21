/**
 * \file block_transporter_metrics_collector.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#include "cosm/fsm/metrics/block_transporter_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::fsm::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_transporter_metrics_collector
 * \ingroup fsm metrics
 *
 * \brief Collector for \ref block_transporter_metrics.
 *
 * Metrics CAN be collected in parallel from robots; concurrent updates to the
 * gathered stats are supported.
 */
class block_transporter_metrics_collector final : public rmetrics::base_collector {
 public:
  /**
   * \param sink The metrics sink to use.
   */
  explicit block_transporter_metrics_collector(
      std::unique_ptr<rmetrics::base_sink> sink);

  /* base_collector overrides */
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;
  const rmetrics::base_data* data(void) const override { return &m_data; }

#if defined(COSM_PAL_TARGET_ROS)
  void collect(const block_transporter_metrics_data& data) { m_data += data; }
#endif

 private:
  /* clang-format off */
  block_transporter_metrics_data m_data{};
  /* clang-format on */
};

} /* namespace cosm::fsm::metrics */
