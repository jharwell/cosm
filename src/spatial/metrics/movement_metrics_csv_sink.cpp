/**
 * \file movement_metrics_csv_sink.cpp
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
#include "cosm/spatial/metrics/movement_metrics_csv_sink.hpp"

#include "cosm/spatial/metrics/movement_metrics.hpp"
#include "cosm/spatial/metrics/movement_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
movement_metrics_csv_sink::movement_metrics_csv_sink(
    fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> movement_metrics_csv_sink::csv_header_cols(
const rmetrics::base_data*) const {
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

boost::optional<std::string> movement_metrics_csv_sink::csv_line_build(
    const rmetrics::base_data* data,
    const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }

  auto* d = dynamic_cast<const movement_metrics_data*>(data);

  std::string line;
  /* homing motion */
  line += csv_entry_domavg(d->interval[movement_category::ekHOMING].distance.load(),
                           d->interval[movement_category::ekHOMING].n_robots);
  line += csv_entry_domavg(d->cum[movement_category::ekHOMING].distance.load(),
                           d->cum[movement_category::ekHOMING].n_robots);

  line += csv_entry_domavg(d->interval[movement_category::ekHOMING].velocity.load(),
                           d->interval[movement_category::ekHOMING].n_robots);
  line += csv_entry_domavg(d->cum[movement_category::ekHOMING].velocity.load(),
                           d->cum[movement_category::ekHOMING].n_robots);

  /* exploring motion */
  line += csv_entry_domavg(d->interval[movement_category::ekEXPLORING].distance.load(),
                           d->interval[movement_category::ekEXPLORING].n_robots);
  line += csv_entry_domavg(d->cum[movement_category::ekEXPLORING].distance.load(),
                           d->cum[movement_category::ekEXPLORING].n_robots);

  line += csv_entry_domavg(d->interval[movement_category::ekEXPLORING].velocity.load(),
                           d->interval[movement_category::ekEXPLORING].n_robots);
  line += csv_entry_domavg(d->cum[movement_category::ekEXPLORING].velocity.load(),
                           d->cum[movement_category::ekEXPLORING].n_robots);

  /* all motion */
  line += csv_entry_domavg(d->interval[movement_category::ekALL].distance.load(),
                           d->interval[movement_category::ekALL].n_robots);
  line += csv_entry_domavg(d->cum[movement_category::ekALL].distance.load(),
                           d->cum[movement_category::ekALL].n_robots);

  line += csv_entry_domavg(d->interval[movement_category::ekALL].velocity.load(),
                           d->interval[movement_category::ekALL].n_robots);
  line += csv_entry_domavg(d->cum[movement_category::ekALL].velocity.load(),
                           d->cum[movement_category::ekALL].n_robots,
                           true);

  return boost::make_optional(line);
} /* csv_line_build() */


NS_END(metrics, spatial, cosm);
