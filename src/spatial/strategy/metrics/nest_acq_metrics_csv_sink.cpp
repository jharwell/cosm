/**
 * \file nest_acq_metrics_csv_sink.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include "cosm/spatial/strategy/metrics/nest_acq_metrics_csv_sink.hpp"

#include "cosm/spatial/strategy/metrics/nest_acq_metrics_data.hpp"
#include "cosm/spatial/strategy/nest_acq/random_thresh.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, strategy, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
nest_acq_metrics_csv_sink::nest_acq_metrics_csv_sink(
        fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> nest_acq_metrics_csv_sink::csv_header_cols(
    const rmetrics::base_data*) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_avg_random_thresh",
    "cum_avg_random_thresh",
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

boost::optional<std::string> nest_acq_metrics_csv_sink::csv_line_build(
const rmetrics::base_data* data,
      const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }

  auto* d = static_cast<const nest_acq_metrics_data*>(data);

  std::string line;

  line += csv_entry_domavg(d->interval.random_thresh.load(),
                           d->interval.n_random_thresh);
  line += csv_entry_domavg(d->cum.random_thresh.load(),
                           d->cum.n_random_thresh,
                           true);

  return boost::make_optional(line);
} /* csv_line_build() */


NS_END(metrics, strategy, spatial, cosm);
