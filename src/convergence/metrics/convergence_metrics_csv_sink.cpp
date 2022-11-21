/**
 * \file convergence_metrics_csv_sink.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/convergence/metrics/convergence_metrics_csv_sink.hpp"

#include "cosm/convergence/metrics/convergence_metrics_data.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::convergence::metrics {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
convergence_metrics_csv_sink::convergence_metrics_csv_sink(
    fs::path fpath_no_ext,
    const rmetrics::output_mode& mode,
    const rtypes::timestep& interval)
    : csv_sink(fpath_no_ext, mode, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
convergence_metrics_csv_sink::csv_header_cols(const rmetrics::base_data*) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "conv_epsilon",
    "int_avg_interact_deg_raw",
    "int_avg_interact_deg_norm",
    "int_avg_interact_deg_converged",
    "int_avg_ang_order_raw",
    "int_avg_ang_order_norm",
    "int_avg_ang_order_converged",
    "int_avg_pos_entropy_raw",
    "int_avg_pos_entropy_norm",
    "int_avg_pos_entropy_converged",
    "int_avg_task_dist_entropy_raw",
    "int_avg_task_dist_entropy_norm",
    "int_avg_task_dist_entropy_converged",
    "int_avg_velocity_raw",
    "int_avg_velocity_norm",
    "int_avg_velocity_converged"
    /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

boost::optional<std::string>
convergence_metrics_csv_sink::csv_line_build(const rmetrics::base_data* data,
                                             const rtypes::timestep& t) {
  if (!ready_to_flush(t)) {
    return boost::none;
  }

  auto* d = dynamic_cast<const convergence_metrics_data*>(data);

  std::string line;
  line += rcppsw::to_string(d->conv_epsilon) + separator();

  /*
   * There are no cumulative metrics, because we also output dt values, which
   * are almost the same thing, and much more useful when calculating
   * convergence.
   */
  line += csv_entry_intavg(d->interact.raw);
  line += csv_entry_intavg(d->interact.norm);
  line += csv_entry_intavg(d->interact.converged);

  line += csv_entry_intavg(d->order.raw);
  line += csv_entry_intavg(d->order.norm);
  line += csv_entry_intavg(d->order.converged);

  line += csv_entry_intavg(d->tdist_ent.raw);
  line += csv_entry_intavg(d->tdist_ent.norm);
  line += csv_entry_intavg(d->tdist_ent.converged);

  line += csv_entry_intavg(d->pos_ent.raw);
  line += csv_entry_intavg(d->pos_ent.norm);
  line += csv_entry_intavg(d->pos_ent.converged);

  line += csv_entry_intavg(d->velocity.raw);
  line += csv_entry_intavg(d->velocity.norm);
  line += csv_entry_intavg(d->velocity.converged, true);

  return boost::make_optional(line);
} /* csv_line_build() */

} /* namespace cosm::convergence::metrics */
