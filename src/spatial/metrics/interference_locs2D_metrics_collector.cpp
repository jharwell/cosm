/**
 * \file interference_locs2D_metrics_collector.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/metrics/interference_locs2D_metrics_collector.hpp"

#include "cosm/spatial/metrics/interference_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::spatial::metrics {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
interference_locs2D_metrics_collector::interference_locs2D_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink,
    const rmath::vector2z& dims)
    : grid2D_metrics_collector(std::move(sink), dims) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void interference_locs2D_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const interference_metrics&>(metrics);
  inc_total_count();

  if (auto loc = m.interference_loc3D()) {
    inc_cell_count(loc->to_2D());
  }
} /* collect() */

} /* namespace cosm::spatial::metrics */
