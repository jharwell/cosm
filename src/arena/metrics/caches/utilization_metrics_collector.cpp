/**
 * \file utilization_metrics_collector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/arena/metrics/caches/utilization_metrics_collector.hpp"

#include "cosm/arena/metrics/caches/utilization_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::metrics::caches {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
utilization_metrics_collector::utilization_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void utilization_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const utilization_metrics&>(metrics);

  m_data.interval.n_pickups += m.total_block_pickups();
  m_data.interval.n_drops += m.total_block_drops();
  m_data.interval.n_blocks += m.n_blocks();

  m_data.cum.n_pickups += m.total_block_pickups();
  m_data.cum.n_drops += m.total_block_drops();
  m_data.cum.n_blocks += m.n_blocks();

  ++m_data.interval.cache_count;
  ++m_data.cum.cache_count;
} /* collect() */

void utilization_metrics_collector::reset_after_interval(void) {
  m_data.interval.n_blocks = 0;
  m_data.interval.n_pickups = 0;
  m_data.interval.n_drops = 0;
  m_data.interval.cache_count = 0;
} /* resedt_after_interval() */

} /* namespace cosm::arena::metrics::caches */
