/**
 * \file location_metrics_collector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/arena/metrics/caches/location_metrics_collector.hpp"

#include "cosm/arena/metrics/caches/location_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, metrics, caches);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void location_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const location_metrics&>(metrics);
  inc_total_count();
  inc_cell_count(m.location());
} /* collect() */

NS_END(caches, metrics, arena, cosm);
