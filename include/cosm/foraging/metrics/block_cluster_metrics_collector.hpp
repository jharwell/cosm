/**
 * \file block_cluster_metrics_collector.hpp
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
#include "cosm/foraging/metrics/block_cluster_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_cluster_metrics_collector
 * \ingroup foraging metrics
 *
 * \brief Collector for \ref block_cluster_metrics.
 */
class block_cluster_metrics_collector final : public rmetrics::base_collector {
 public:
  /**
   * \param sink The metrics sink to use.
   */
  block_cluster_metrics_collector(
      std::unique_ptr<rmetrics::base_sink> sink,
      size_t n_clusters);

  /* base_collector overrides */
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;
  const rmetrics::base_data* data(void) const override { return &m_data; }


#if defined(COSM_PAL_TARGET_ROS)
  void collect(const block_cluster_metrics_data& data) { m_data += data; }
#endif

 private:
  /* clang-format off */
  block_cluster_metrics_data m_data;
  /* clang-format on */
};

} /* namespace cosm::foraging::metrics */
