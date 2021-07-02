/**
 * \file movement_metrics_collector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/metrics/movement_metrics_collector.hpp"

#include "cosm/spatial/metrics/movement_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
movement_metrics_collector::movement_metrics_collector(
    std::unique_ptr<rmetrics::base_metrics_sink> sink)
    : base_metrics_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void movement_metrics_collector::collect(const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const movement_metrics&>(metrics);
  auto all_filter = [&](double) { return true; };
  auto other_filter = [&](double dist) { return dist > 0.0; };
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
    ++m_data.cum[i].n_robots;
    auto cum_dist = m_data.cum[i].distance.load();
    auto int_dist = m_data.interval[i].distance.load();
    auto cum_vel = m_data.cum[i].velocity.load();
    auto int_vel = m_data.interval[i].velocity.load();

    m_data.cum[i].distance.compare_exchange_strong(cum_dist, cum_dist + ts_dist);
    m_data.interval[i].distance.compare_exchange_strong(int_dist, int_dist + ts_dist);
    m_data.cum[i].velocity.compare_exchange_strong(cum_vel, cum_vel + ts_vel);
    m_data.interval[i].velocity.compare_exchange_strong(int_vel, int_vel + ts_vel);
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
