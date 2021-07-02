/**
 * \file convergence_metrics_csv_sink.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/convergence/metrics/convergence_metrics_csv_sink.hpp"
#include "cosm/convergence/metrics/convergence_metrics_data.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, convergence, metrics);

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
convergence_metrics_csv_sink::csv_header_cols(
const rmetrics::base_metrics_data*) const {
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

boost::optional<std::string> convergence_metrics_csv_sink::csv_line_build(
  const rmetrics::base_metrics_data* data,
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

NS_END(metrics, convergence, cosm);
