/**
 * \file explore_locs2D_metrics_collector.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
namespace cosm::spatial::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class explore_locs2D_metrics_collector
 * \ingroup spatial metrics
 *
 * \brief Collector for robot exploration trajectories for goal acquisition,
 * which is collected as a 2D array, and needs its own collector separate from
 * the \ref goal_acq_metrics_collector (1 .csv per collector).
 *
 * Metrics CAN be collected concurrently if the calling context guarantees that
 * no two robots will have the same discrete location. Otherwise, serial
 * collection is required.
 */
class explore_locs2D_metrics_collector final : public rdmetrics::grid2D_metrics_collector {
 public:
  /**
   * \param sink The metrics sink to use.
   * \param dims Dimensions of arena.
   */
  explore_locs2D_metrics_collector(
      std::unique_ptr<rmetrics::base_sink> sink,
      const rmath::vector2z& dims);

  void collect(const rmetrics::base_metrics& metrics) override;
};

} /* namespace cosm::spatial::metrics */
