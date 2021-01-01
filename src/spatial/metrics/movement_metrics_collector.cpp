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
    const std::string& ofname_stem,
    const rtypes::timestep& interval)
    : base_metrics_collector(ofname_stem,
                             interval,
                             rmetrics::output_mode::ekAPPEND) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> movement_metrics_collector::csv_header_cols(void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_avg_distance_homing",
    "cum_avg_distance_homing",
    "int_avg_velocity_homing",
    "cum_avg_velocity_homing",
    "int_avg_distance_exploring",
    "cum_avg_distance_exploring",
    "int_avg_velocity_exploring",
    "cum_avg_velocity_exploring",
    "int_avg_distance_all",
    "cum_avg_distance_all",
    "int_avg_velocity_all",
    "cum_avg_velocity_all"
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void movement_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

boost::optional<std::string> movement_metrics_collector::csv_line_build(void) {
  if (!(timestep() % interval() == 0)) {
    return boost::none;
  }
  std::string line;
  /* homing motion */
  line +=
      csv_entry_domavg(m_interval[movement_category::ekHOMING].distance.load(),
                       m_interval[movement_category::ekHOMING].n_robots);
  line += csv_entry_domavg(m_cum[movement_category::ekHOMING].distance.load(),
                           m_cum[movement_category::ekHOMING].n_robots);

  line +=
      csv_entry_domavg(m_interval[movement_category::ekHOMING].velocity.load(),
                       m_interval[movement_category::ekHOMING].n_robots);
  line += csv_entry_domavg(m_cum[movement_category::ekHOMING].velocity.load(),
                           m_cum[movement_category::ekHOMING].n_robots);

  /* exploring motion */
  line +=
      csv_entry_domavg(m_interval[movement_category::ekEXPLORING].distance.load(),
                       m_interval[movement_category::ekEXPLORING].n_robots);
  line += csv_entry_domavg(m_cum[movement_category::ekEXPLORING].distance.load(),
                           m_cum[movement_category::ekEXPLORING].n_robots);

  line +=
      csv_entry_domavg(m_interval[movement_category::ekEXPLORING].velocity.load(),
                       m_interval[movement_category::ekEXPLORING].n_robots);
  line += csv_entry_domavg(m_cum[movement_category::ekEXPLORING].velocity.load(),
                           m_cum[movement_category::ekEXPLORING].n_robots);

  /* all motion */
  line += csv_entry_domavg(m_interval[movement_category::ekALL].distance.load(),
                           m_interval[movement_category::ekALL].n_robots);
  line += csv_entry_domavg(m_cum[movement_category::ekALL].distance.load(),
                           m_cum[movement_category::ekALL].n_robots);

  line += csv_entry_domavg(m_interval[movement_category::ekALL].velocity.load(),
                           m_interval[movement_category::ekALL].n_robots);
  line += csv_entry_domavg(m_cum[movement_category::ekALL].velocity.load(),
                           m_cum[movement_category::ekALL].n_robots,
                           true);

  return boost::make_optional(line);
} /* csv_line_build() */

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
    ++m_interval[i].n_robots;
    ++m_cum[i].n_robots;
    auto cum_dist = m_cum[i].distance.load();
    auto int_dist = m_interval[i].distance.load();
    auto cum_vel = m_cum[i].velocity.load();
    auto int_vel = m_interval[i].velocity.load();

    m_cum[i].distance.compare_exchange_strong(cum_dist, cum_dist + ts_dist);
    m_interval[i].distance.compare_exchange_strong(int_dist, int_dist + ts_dist);
    m_cum[i].velocity.compare_exchange_strong(cum_vel, cum_vel + ts_vel);
    m_interval[i].velocity.compare_exchange_strong(int_vel, int_vel + ts_vel);
  } /* for(i..) */
} /* collect() */

void movement_metrics_collector::reset_after_interval(void) {
  for (size_t i = 0; i < movement_category::ekMAX; ++i) {
    m_interval[i].distance = 0.0;
    m_interval[i].velocity = 0.0;
    m_interval[i].n_robots = 0;
  } /* for(i..) */
} /* reset_after_interval() */

NS_END(metrics, spatial, cosm);
