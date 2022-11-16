/**
 * \file execution_metrics_csv_sink.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/metrics/execution_metrics_csv_sink.hpp"

#include "cosm/ta/metrics/execution_metrics_data.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
execution_metrics_csv_sink::execution_metrics_csv_sink(
    fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
execution_metrics_csv_sink::csv_header_cols(const rmetrics::base_data*) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_avg_exec_time",
    "cum_avg_exec_time",
    "int_avg_interface_time",
    "cum_avg_interface_time",
    "int_avg_exec_estimate",
    "cum_avg_exec_estimate",
    "int_avg_interface_estimate",
    "cum_avg_interface_estimate",
    "int_avg_abort_count",
    "cum_avg_abort_count",
    "int_avg_complete_count",
    "cum_avg_complete_count",
    "int_avg_interface_count",
    "cum_avg_interface_count"
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

boost::optional<std::string>
execution_metrics_csv_sink::csv_line_build(const rmetrics::base_data* data,
                                           const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }
  auto* d = static_cast<const execution_metrics_data*>(data);

  size_t int_n_allocs = d->interval.complete_count + d->interval.abort_count;
  size_t cum_n_allocs = d->cum.complete_count + d->cum.abort_count;
  std::string line;

  line += csv_entry_domavg(d->interval.exec_time, int_n_allocs);
  line += csv_entry_domavg(d->cum.exec_time, cum_n_allocs);
  line += csv_entry_domavg(d->interval.interface_time, int_n_allocs);
  line += csv_entry_domavg(d->cum.interface_time, cum_n_allocs);
  line += csv_entry_domavg(d->interval.exec_estimate, int_n_allocs);
  line += csv_entry_domavg(d->cum.exec_estimate, cum_n_allocs);

  line += csv_entry_domavg(d->interval.interface_estimate, int_n_allocs);
  line += csv_entry_domavg(d->cum.interface_estimate, cum_n_allocs);

  line += csv_entry_intavg(d->interval.abort_count);
  line += csv_entry_tsavg(d->cum.abort_count, t);
  line += csv_entry_intavg(d->interval.complete_count);
  line += csv_entry_tsavg(d->cum.complete_count, t);
  line += csv_entry_intavg(d->interval.interface_count);
  line += csv_entry_tsavg(d->cum.interface_count, t, true);

  return boost::make_optional(line);
} /* store_foraging_stats() */

NS_END(metrics, ta, cosm);
