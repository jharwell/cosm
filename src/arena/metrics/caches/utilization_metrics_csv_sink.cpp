/**
 * \file utilization_metrics_csv_sink.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/arena/metrics/caches/utilization_metrics_csv_sink.hpp"

#include "cosm/arena/metrics/caches/utilization_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, metrics, caches);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
utilization_metrics_csv_sink::utilization_metrics_csv_sink(
    fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : csv_sink(std::move(fpath_no_ext), mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
utilization_metrics_csv_sink::csv_header_cols(const rmetrics::base_data*) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_avg_blocks",
    "cum_avg_blocks",
    "int_avg_pickups",
    "cum_avg_pickups",
    "int_avg_drops" ,
    "cum_avg_drops" ,
    "int_avg_caches",
    "cum_avg_caches"
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

boost::optional<std::string>
utilization_metrics_csv_sink::csv_line_build(const rmetrics::base_data* data,
                                             const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }
  std::string line;

  auto* d = static_cast<const utilization_metrics_data*>(data);

  line += csv_entry_domavg(d->interval.n_blocks, d->interval.cache_count);
  line += csv_entry_domavg(d->cum.n_blocks, d->cum.cache_count);
  line += csv_entry_domavg(d->interval.n_pickups, d->interval.cache_count);
  line += csv_entry_domavg(d->cum.n_pickups, d->cum.cache_count);
  line += csv_entry_domavg(d->interval.n_drops, d->interval.cache_count);
  line += csv_entry_domavg(d->cum.n_drops, d->cum.cache_count);
  line += csv_entry_intavg(d->interval.cache_count);
  line += csv_entry_tsavg(d->cum.cache_count, t, true);

  return boost::make_optional(line);
} /* csv_line_build() */

NS_END(caches, metrics, arena, cosm);
