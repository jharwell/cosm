/**
 * \file dist3D_pos_metrics_collector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/metrics/dist3D_pos_metrics_collector.hpp"

#include "rcppsw/metrics/base_sink.hpp"

#include "cosm/spatial/metrics/dist3D_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
dist3D_pos_metrics_collector::dist3D_pos_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink,
    const rmath::vector3z& dims)
    : grid3D_metrics_collector(std::move(sink), dims) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void dist3D_pos_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const dist3D_metrics&>(metrics);
  inc_total_count();
  inc_cell_count(m.dpos3D());
} /* collect() */

NS_END(metrics, spatial, cosm);
