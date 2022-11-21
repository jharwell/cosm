/**
 * \file block_transportee_metrics_csv_sink.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/metrics/block_transportee_metrics_csv_sink.hpp"

#include "cosm/foraging/metrics/block_transportee_metrics_data.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::metrics {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_transportee_metrics_csv_sink::block_transportee_metrics_csv_sink(
    fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> block_transportee_metrics_csv_sink::csv_header_cols(
    const rmetrics::base_data*) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "cum_transported",
    "cum_ramp_transported",
    "cum_cube_transported",
    "int_avg_transported",
    "cum_avg_transported",
    "int_avg_cube_transported",
    "cum_avg_cube_transported",
    "int_avg_ramp_transported",
    "cum_avg_ramp_transported",
    "int_avg_transporters",
    "cum_avg_transporters",
    "int_avg_transport_time",
    "cum_avg_transport_time",
    "int_avg_initial_wait_time",
    "cum_avg_initial_wait_time"
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

boost::optional<std::string> block_transportee_metrics_csv_sink::csv_line_build(
    const rmetrics::base_data* data,
    const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }

  auto* d = static_cast<const block_transportee_metrics_data*>(data);

  std::string line;

  line += rcppsw::to_string(d->cum.n_transported) + separator();
  line += rcppsw::to_string(d->cum.n_ramp_transported) + separator();
  line += rcppsw::to_string(d->cum.n_cube_transported) + separator();

  line += csv_entry_intavg(d->interval.n_transported);
  line += csv_entry_tsavg(d->cum.n_transported, t);

  line += csv_entry_intavg(d->interval.n_cube_transported);
  line += csv_entry_tsavg(d->cum.n_cube_transported, t);
  line += csv_entry_intavg(d->interval.n_ramp_transported);
  line += csv_entry_tsavg(d->cum.n_ramp_transported, t);
  line += csv_entry_domavg(d->interval.n_transporters, d->interval.n_transported);
  line += csv_entry_domavg(d->cum.n_transporters, d->cum.n_transported);

  line += csv_entry_domavg(d->interval.transport_time, d->interval.n_transported);
  line += csv_entry_domavg(d->cum.transport_time, d->cum.n_transported);

  /*
   * If it is 0, then no blocks were collected this interval, so the initial
   * wait time is infinite.
   */
  if (d->interval.initial_wait_time > 0) {
    line += csv_entry_domavg(d->interval.initial_wait_time,
                             d->interval.n_transported);
  } else {
    line += "inf" + separator();
  }

  /*
   * If it is 0, then no blocks were collected yet, so the initial wait time is
   * infinite.
   */
  if (d->cum.initial_wait_time > 0) {
    line +=
        csv_entry_domavg(d->cum.initial_wait_time, d->cum.n_transported, true);
  } else {
    line += "inf";
  }

  return boost::make_optional(line);
} /* csv_line_build() */

} /* namespace cosm::foraging::metrics */
