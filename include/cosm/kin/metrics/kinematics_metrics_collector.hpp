/**
 * \file kinematics_metrics_collector.hpp
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
#include "rcppsw/spatial/euclidean_dist.hpp"

#include "cosm/cosm.hpp"
#include "cosm/kin/metrics/kinematics_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::kin::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class kinematics_metrics_collector
 * \ingroup kin metrics
 *
 * \brief Collector for \ref kinematics_metrics.
 *
 * Metrics CAN be collected in parallel from robots; concurrent updates to the
 * gathered stats are supported.
 */
class kinematics_metrics_collector final : public rer::client<kinematics_metrics_collector>,
                                           public rmetrics::base_collector {
 public:
  /**
   * \param sink The metrics sink to use.
   *
   * \param n_robots How many robotsare being tracked.
   */
  kinematics_metrics_collector(std::unique_ptr<rmetrics::base_sink> sink,
                               size_t n_robots,
                               size_t n_contexts);

  /* base_collector overrides */
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;
  const rmetrics::base_data* data(void) const override { return &m_data; }

#if defined(COSM_PAL_TARGET_ROS)
  void collect(const kinematics_metrics_data& data) { m_data += data; }
#endif

 private:
  /* clang-format off */
  kinematics_metrics_data m_data;
  /* clang-format on */
};

} /* namespace cosm::kin::metrics */
