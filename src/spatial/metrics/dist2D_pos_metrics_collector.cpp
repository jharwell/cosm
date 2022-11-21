/**
 * \file dist2D_pos_metrics_collector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/metrics/dist2D_pos_metrics_collector.hpp"

#include "rcppsw/metrics/base_sink.hpp"

#include "cosm/spatial/metrics/dist2D_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::spatial::metrics {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dist2D_pos_metrics_collector::dist2D_pos_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink,
    const rmath::vector2z& dims)
    : grid2D_metrics_collector(std::move(sink), dims) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void dist2D_pos_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const dist2D_metrics&>(metrics);
  inc_total_count();
  inc_cell_count(m.dpos2D());
} /* collect() */

} /* namespace cosm::spatial::metrics */
