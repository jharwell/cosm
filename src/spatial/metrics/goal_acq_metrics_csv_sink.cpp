/**
 * \file goal_acq_metrics_csv_sink.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/metrics/goal_acq_metrics_csv_sink.hpp"

#include "cosm/spatial/metrics/goal_acq_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::spatial::metrics {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
goal_acq_metrics_csv_sink::goal_acq_metrics_csv_sink(
    fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
goal_acq_metrics_csv_sink::csv_header_cols(const rmetrics::base_data*) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_avg_acquiring_goal",
    "cum_avg_acquiring_goal",
    "int_avg_vectoring_to_goal",
    "cum_avg_vectoring_to_goal",
    "int_avg_true_exploring_for_goal",
    "cum_avg_true_exploring_for_goal",
    "int_avg_false_exploring_for_goal",
    "cum_avg_false_exploring_for_goal",
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

boost::optional<std::string>
goal_acq_metrics_csv_sink::csv_line_build(const rmetrics::base_data* data,
                                          const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }

  auto* d = static_cast<const goal_acq_metrics_data*>(data);
  std::string line;
  line += csv_entry_intavg(ral::mt_load(d->interval.n_acquiring_goal));
  line += csv_entry_tsavg(ral::mt_load(d->cum.n_acquiring_goal), t);
  line += csv_entry_intavg(ral::mt_load(d->interval.n_vectoring_to_goal));
  line += csv_entry_tsavg(ral::mt_load(d->cum.n_vectoring_to_goal), t);
  line += csv_entry_intavg(ral::mt_load(d->interval.n_true_exploring_for_goal));
  line += csv_entry_tsavg(ral::mt_load(d->cum.n_true_exploring_for_goal), t);
  line += csv_entry_intavg(ral::mt_load(d->interval.n_false_exploring_for_goal));
  line +=
      csv_entry_tsavg(ral::mt_load(d->cum.n_false_exploring_for_goal), t, true);
  return boost::make_optional(line);
} /* csv_line_build() */

} /* namespace cosm::spatial::metrics */
