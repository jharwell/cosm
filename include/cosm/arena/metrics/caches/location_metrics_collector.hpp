/**
 * \file location_metrics_collector.hpp
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

#include "rcppsw/ds/metrics/grid2D_metrics_collector.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, metrics, caches);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class location_metrics_collector
 * \ingroup arena metrics caches
 *
 * \brief Collector for \ref location_metrics.
 *
 * Metrics MUST be collected serially; concurrent updates to the gathered stats
 * are not supported.
 */
class location_metrics_collector final :
    public rdmetrics::grid2D_metrics_collector {
 public:
  /**
   * \param sink The metrics sink to use.
   * \param dims Dimensions of arena.
   */
  location_metrics_collector(
      std::unique_ptr<rmetrics::base_sink> sink,
      const rmath::vector2z& dims)
      : grid2D_metrics_collector(std::move(sink), dims) {}

  void collect(const rmetrics::base_metrics& metrics) override;
};

NS_END(caches, metrics, arena, cosm);

