/**
 * \file dist2D_pos_metrics_collector.hpp
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

#include "rcppsw/ds/metrics/grid2D_metrics_collector.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dist2D_pos_metrics_collector
 * \ingroup rcppsw metrics swarm spatial
 *
 * \brief Collector for \ref dist2D_metrics.
 *
 * Metrics MUST be collected serially; concurrent updates to the gathered stats
 * are not supported.
 */
class dist2D_pos_metrics_collector final : public rdmetrics::grid2D_metrics_collector {

 public:
  /**
   * \param sink The metrics sink to use.
   * \param dims Dimensions of arena.
   */
  dist2D_pos_metrics_collector(
      std::unique_ptr<rmetrics::base_sink> sink,
      const rmath::vector2z& dims);

  void collect(const rmetrics::base_metrics& metrics) override;
};

NS_END(metrics, spatial, cosm);
