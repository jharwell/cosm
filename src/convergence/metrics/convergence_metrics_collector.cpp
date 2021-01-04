/**
 * \file convergence_metrics_collector.cpp
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
#include "cosm/convergence/metrics/convergence_metrics_collector.hpp"

#include "cosm/convergence/metrics/convergence_metrics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, convergence, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
convergence_metrics_collector::convergence_metrics_collector(
    const std::string& ofname_stem,
    const rtypes::timestep& interval)
    : base_metrics_collector(ofname_stem,
                             interval,
                             rmetrics::output_mode::ekAPPEND) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
convergence_metrics_collector::csv_header_cols(void) const {
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

boost::optional<std::string> convergence_metrics_collector::csv_line_build(void) {
  if (!(timestep() % interval() == 0)) {
    return boost::none;
  }
  std::string line;
  line += rcppsw::to_string(m_conv_epsilon) + separator();

  /*
   * There are no cumulative metrics, because we also output dt values, which
   * are almost the same thing, and much more useful when calculating
   * convergence.
   */
  line += csv_entry_intavg(m_interact_stats.raw);
  line += csv_entry_intavg(m_interact_stats.norm);
  line += csv_entry_intavg(m_interact_stats.converged);

  line += csv_entry_intavg(m_order_stats.raw);
  line += csv_entry_intavg(m_order_stats.norm);
  line += csv_entry_intavg(m_order_stats.converged);

  line += csv_entry_intavg(m_tdist_ent_stats.raw);
  line += csv_entry_intavg(m_tdist_ent_stats.norm);
  line += csv_entry_intavg(m_tdist_ent_stats.converged);

  line += csv_entry_intavg(m_pos_ent_stats.raw);
  line += csv_entry_intavg(m_pos_ent_stats.norm);
  line += csv_entry_intavg(m_pos_ent_stats.converged);

  line += csv_entry_intavg(m_velocity_stats.raw);
  line += csv_entry_intavg(m_velocity_stats.norm);
  line += csv_entry_intavg(m_velocity_stats.converged, true);

  return boost::make_optional(line);
} /* csv_line_build() */

void convergence_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const convergence_metrics&>(metrics);
  /*
   * Captured here, rather than as a constructor parameter in order to allow for
   * temporally varying convergence thresholds in the future if desired.
   */
  m_conv_epsilon = m.swarm_conv_epsilon();

  auto status = m.swarm_interactivity();
  m_interact_stats.raw += std::get<0>(status);
  m_interact_stats.norm += std::get<1>(status);
  m_interact_stats.converged += static_cast<uint>(std::get<2>(status));

  status = m.swarm_angular_order();
  m_order_stats.raw += std::get<0>(status);
  m_order_stats.norm += std::get<1>(status);
  m_order_stats.converged += static_cast<uint>(std::get<2>(status));

  status = m.swarm_positional_entropy();
  m_pos_ent_stats.raw += std::get<0>(status);
  m_pos_ent_stats.norm += std::get<1>(status);
  m_pos_ent_stats.converged += static_cast<uint>(std::get<2>(status));

  status = m.swarm_task_dist_entropy();
  m_tdist_ent_stats.raw += std::get<0>(status);
  m_tdist_ent_stats.norm += std::get<1>(status);
  m_tdist_ent_stats.converged += static_cast<uint>(std::get<2>(status));

  status = m.swarm_velocity();
  m_velocity_stats.raw += std::get<0>(status);
  m_velocity_stats.norm += std::get<1>(status);
  m_velocity_stats.converged += static_cast<uint>(std::get<2>(status));
} /* collect() */

void convergence_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

void convergence_metrics_collector::reset_after_interval(void) {
  m_interact_stats = { 0.0, 0.0, 0 };
  m_order_stats = { 0.0, 0.0, 0 };
  m_pos_ent_stats = { 0.0, 0.0, 0 };
  m_tdist_ent_stats = { 0.0, 0.0, 0 };
  m_velocity_stats = { 0.0, 0.0, 0 };
} /* reset_after_interval() */

NS_END(metrics, convergence, cosm);
