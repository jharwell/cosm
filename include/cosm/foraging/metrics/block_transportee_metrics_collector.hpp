/**
 * \file block_transportee_metrics_collector.hpp
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

#include "cosm/foraging/metrics/block_transportee_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_transportee_metrics_collector
 * \ingroup foraging metrics
 *
 * \brief Collector for \ref block_transportee_metrics.
 *
 * Metrics CAN be collected in parallel from robots; concurrent updates to the
 * gathered stats are supported.
 */
class block_transportee_metrics_collector final : public rmetrics::base_collector {
 public:
  /**
   * \param sink The metrics sink to use.
   */
  explicit block_transportee_metrics_collector(
      std::unique_ptr<rmetrics::base_sink> sink);

  /* base_collector overrides */
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;
  const rmetrics::base_data* data(void) const override {
    return &m_data;
  }
#if defined(COSM_PAL_TARGET_ROS)
  void collect(const block_transportee_metrics_data& data) { m_data += data; }
#endif

  size_t cum_transported(void) const { return m_data.cum.n_transported; }

 private:
  /* clang-format off */
  block_transportee_metrics_data m_data{};
  /* clang-format on */
};

NS_END(metrics, foraging, cosm);
