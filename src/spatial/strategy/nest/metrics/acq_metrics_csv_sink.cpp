/**
 * \file acq_metrics_csv_sink.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/nest/metrics/acq_metrics_csv_sink.hpp"

#include "cosm/spatial/strategy/nest/metrics/acq_metrics_data.hpp"
#include "cosm/spatial/strategy/nest/acq/random_thresh.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, strategy, nest, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
acq_metrics_csv_sink::acq_metrics_csv_sink(
    fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
acq_metrics_csv_sink::csv_header_cols(const rmetrics::base_data*) const {
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

boost::optional<std::string>
acq_metrics_csv_sink::csv_line_build(const rmetrics::base_data* data,
                                          const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }

  auto* d = static_cast<const acq_metrics_data*>(data);

  std::string line;

  line += csv_entry_domavg(ral::mt_load(d->interval.random_thresh),
                           d->interval.n_random_thresh);
  line += csv_entry_domavg(
      ral::mt_load(d->cum.random_thresh), d->cum.n_random_thresh, true);

  return boost::make_optional(line);
} /* csv_line_build() */

NS_END(metrics, nest, strategy, spatial, cosm);
