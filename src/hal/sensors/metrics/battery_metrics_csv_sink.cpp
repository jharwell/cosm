/**
 * \file battery_metrics_csv_sink.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/sensors/metrics/battery_metrics_csv_sink.hpp"

#include "cosm/hal/sensors/metrics/battery_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, sensors, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
battery_metrics_csv_sink::battery_metrics_csv_sink(
    fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : ER_CLIENT_INIT("cosm.hal.sensors.metrics.battery_metrics_csv_sink"),
      csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> battery_metrics_csv_sink::csv_header_cols(
    const rmetrics::base_data*) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_avg_percentage",
    "cum_avg_percentage",
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

boost::optional<std::string> battery_metrics_csv_sink::csv_line_build(
    const rmetrics::base_data* data,
    const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }
  std::string line;

  auto* d = static_cast<const battery_metrics_data*>(data);

  line += csv_entry_domavg(d->interval.percentage,
                           d->interval.n_robots);

  line += csv_entry_domavg(d->cum.percentage,
                           d->cum.n_robots,
                           true);

  return boost::make_optional(line);
} /* csv_line_build() */

NS_END(metrics, sensors, hal, cosm);
