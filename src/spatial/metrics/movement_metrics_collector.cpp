/**
 * \file movement_metrics_collector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/metrics/movement_metrics_collector.hpp"

#include <functional>

#include "cosm/spatial/metrics/movement_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
movement_metrics_collector::movement_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void movement_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const movement_metrics&>(metrics);
  auto all_filter = [&](double) noexcept { return true; };
  auto other_filter = [&](double dist) noexcept { return dist > 0.0; };
  std::vector<std::function<bool(double)>> filter_funcs(movement_category::ekMAX);
  filter_funcs[movement_category::ekALL] = all_filter;
  filter_funcs[movement_category::ekHOMING] = other_filter;
  filter_funcs[movement_category::ekEXPLORING] = other_filter;

  for (size_t i = 0; i < movement_category::ekMAX; ++i) {
    auto ienum = static_cast<movement_category>(i);
    auto ts_dist = m.ts_distance(ienum).v();
    auto ts_vel = m.ts_velocity(ienum).length();

    if (!filter_funcs[i](ts_dist)) {
      continue;
    }
    ++m_data.interval[i].n_robots;
    ral::mt_accum(m_data.interval[i].distance, ts_dist);
    ral::mt_accum(m_data.interval[i].velocity, ts_vel);

    ++m_data.cum[i].n_robots;
    ral::mt_accum(m_data.cum[i].distance, ts_dist);
    ral::mt_accum(m_data.cum[i].velocity, ts_vel);
  } /* for(i..) */
} /* collect() */

void movement_metrics_collector::reset_after_interval(void) {
  for (size_t i = 0; i < movement_category::ekMAX; ++i) {
    m_data.interval[i].distance = 0.0;
    m_data.interval[i].velocity = 0.0;
    m_data.interval[i].n_robots = 0;
  } /* for(i..) */
} /* reset_after_interval() */

NS_END(metrics, spatial, cosm);
