/**
 * \file nest_zone_metrics_csv_sink.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/metrics/nest_zone_metrics_csv_sink.hpp"

#include "cosm/spatial/metrics/nest_zone_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::spatial::metrics {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
nest_zone_metrics_csv_sink::nest_zone_metrics_csv_sink(
    fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
nest_zone_metrics_csv_sink::csv_header_cols(const rmetrics::base_data*) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_avg_in_nest",
    "int_avg_entered_nest",
    "int_avg_exited_nest",
    "int_avg_nest_duration",
    "cum_avg_in_nest",
    "cum_avg_entered_nest",
    "cum_avg_exited_nest",
    "cum_avg_nest_duration",
    "first_entry_time"
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

boost::optional<std::string>
nest_zone_metrics_csv_sink::csv_line_build(const rmetrics::base_data* data,
                                           const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }

  auto* d = static_cast<const nest_zone_metrics_data*>(data);

  std::string line;

  line += csv_entry_intavg(d->interval.n_in_nest);
  line += csv_entry_intavg(d->interval.n_entered_nest);
  line += csv_entry_intavg(d->interval.n_exited_nest);
  line += csv_entry_intavg(d->interval.nest_duration);

  line += csv_entry_tsavg(d->cum.n_in_nest, t);
  line += csv_entry_tsavg(d->cum.n_entered_nest, t);
  line += csv_entry_tsavg(d->cum.n_exited_nest, t);
  line += csv_entry_tsavg(d->cum.nest_duration, t);
  line += std::to_string(d->cum.first_nest_entry_time);

  return boost::make_optional(line);
} /* csv_line_build() */

} /* namespace cosm::spatial::metrics */
