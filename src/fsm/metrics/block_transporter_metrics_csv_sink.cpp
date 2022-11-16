/**
 * \file block_transporter_metrics_csv_sink.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/fsm/metrics/block_transporter_metrics_csv_sink.hpp"

#include "cosm/fsm/metrics/block_transporter_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, fsm, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_transporter_metrics_csv_sink::block_transporter_metrics_csv_sink(
    fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : ER_CLIENT_INIT("cosm.fsm.metrics.block_transporter_metrics_csv_sink"),
      csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> block_transporter_metrics_csv_sink::csv_header_cols(
    const rmetrics::base_data*) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_avg_phototaxiing_to_goal_including_ca",
    "int_avg_phototaxiing_to_goal_no_ca",
    "cum_avg_phototaxiing_to_goal_including_ca",
    "cum_avg_phototaxiing_to_goal_no_ca",
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

boost::optional<std::string> block_transporter_metrics_csv_sink::csv_line_build(
    const rmetrics::base_data* data,
    const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }
  std::string line;

  auto* d = static_cast<const block_transporter_metrics_data*>(data);

  line += csv_entry_intavg(d->interval.n_phototaxiing_to_goal_including_ca);
  line += csv_entry_intavg(d->interval.n_phototaxiing_to_goal_no_ca);

  line += csv_entry_tsavg(d->cum.n_phototaxiing_to_goal_including_ca, t);
  line += csv_entry_tsavg(d->cum.n_phototaxiing_to_goal_no_ca, t, true);

  return boost::make_optional(line);
} /* csv_line_build() */

NS_END(metrics, fsm, cosm);
