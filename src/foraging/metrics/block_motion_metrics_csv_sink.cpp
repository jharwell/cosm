/**
 * \file block_motion_metrics_csv_sink.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/metrics/block_motion_metrics_csv_sink.hpp"

#include "cosm/foraging/metrics/block_motion_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_motion_metrics_csv_sink::block_motion_metrics_csv_sink(
    fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
block_motion_metrics_csv_sink::csv_header_cols(const rmetrics::base_data*) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_avg_blocks_moved",
    "cum_avg_blocks_moved",
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

boost::optional<std::string>
block_motion_metrics_csv_sink::csv_line_build(const rmetrics::base_data* data,
                                              const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }

  auto* d = static_cast<const block_motion_metrics_data*>(data);

  std::string line;

  line += csv_entry_intavg(d->interval.n_moved);
  line += csv_entry_tsavg(d->cum.n_moved, t, true);

  return boost::make_optional(line);
} /* csv_line_build() */

NS_END(metrics, foraging, cosm);
