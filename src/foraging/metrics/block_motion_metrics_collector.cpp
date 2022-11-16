/**
 * \file block_motion_metrics_collector.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/metrics/block_motion_metrics_collector.hpp"

#include "cosm/foraging/metrics/block_motion_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_motion_metrics_collector::block_motion_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_motion_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = static_cast<const block_motion_metrics&>(metrics);
  m_data.interval.n_moved += m.n_moved();
  m_data.cum.n_moved += m.n_moved();
} /* collect() */

void block_motion_metrics_collector::reset_after_interval(void) {
  m_data.interval.n_moved = 0;
} /* reset_after_interval() */

NS_END(metrics, foraging, cosm);
