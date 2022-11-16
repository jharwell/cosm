/**
 * \file interference_metrics_csv_sink.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/metrics/interference_metrics_csv_sink.hpp"

#include "cosm/spatial/metrics/interference_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
interference_metrics_csv_sink::interference_metrics_csv_sink(
    fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
interference_metrics_csv_sink::csv_header_cols(const rmetrics::base_data*) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_avg_exp_interference",
    "int_avg_entered_interference",
    "int_avg_exited_interference",
    "int_avg_episodes",
    "int_avg_interference_duration",

    "cum_avg_exp_interference",
    "cum_avg_entered_interference",
    "cum_avg_exited_interference",
    "cum_avg_episodes",
    "cum_avg_interference_duration"
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

boost::optional<std::string>
interference_metrics_csv_sink::csv_line_build(const rmetrics::base_data* data,
                                              const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }

  auto* d = static_cast<const interference_metrics_data*>(data);

  std::string line;
  line += csv_entry_intavg(ral::mt_load(d->interval.n_exp_interference));
  line += csv_entry_intavg(ral::mt_load(d->interval.n_entered_interference));
  line += csv_entry_intavg(ral::mt_load(d->interval.n_exited_interference));
  line += csv_entry_intavg(ral::mt_load(d->interval.n_episodes));
  line += csv_entry_domavg(ral::mt_load(d->interval.interference_duration),
                           ral::mt_load(d->interval.n_episodes));

  line += csv_entry_tsavg(ral::mt_load(d->cum.n_exp_interference), t);
  line += csv_entry_tsavg(ral::mt_load(d->cum.n_entered_interference), t);
  line += csv_entry_tsavg(ral::mt_load(d->cum.n_exited_interference), t);
  line += csv_entry_tsavg(ral::mt_load(d->cum.n_episodes), t);
  line += csv_entry_domavg(ral::mt_load(d->cum.interference_duration),
                           ral::mt_load(d->cum.n_episodes),
                           true);

  return boost::make_optional(line);
} /* csv_line_build() */

NS_END(metrics, spatial, cosm);
