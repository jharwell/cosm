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
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void convergence_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const convergence_metrics&>(metrics);
  /*
   * Captured here, rather than as a constructor parameter in order to allow for
   * temporally varying convergence thresholds in the future if desired.
   */
  m_data.conv_epsilon = m.swarm_conv_epsilon();

  auto status = m.swarm_interactivity();
  m_data.interact.raw += std::get<0>(status);
  m_data.interact.norm += std::get<1>(status);
  m_data.interact.converged += static_cast<uint>(std::get<2>(status));

  status = m.swarm_angular_order();
  m_data.order.raw += std::get<0>(status);
  m_data.order.norm += std::get<1>(status);
  m_data.order.converged += static_cast<uint>(std::get<2>(status));

  status = m.swarm_positional_entropy();
  m_data.pos_ent.raw += std::get<0>(status);
  m_data.pos_ent.norm += std::get<1>(status);
  m_data.pos_ent.converged += static_cast<uint>(std::get<2>(status));

  status = m.swarm_task_dist_entropy();
  m_data.tdist_ent.raw += std::get<0>(status);
  m_data.tdist_ent.norm += std::get<1>(status);
  m_data.tdist_ent.converged += static_cast<uint>(std::get<2>(status));

  status = m.swarm_velocity();
  m_data.velocity.raw += std::get<0>(status);
  m_data.velocity.norm += std::get<1>(status);
  m_data.velocity.converged += static_cast<uint>(std::get<2>(status));
} /* collect() */

void convergence_metrics_collector::reset_after_interval(void) {
  m_data.interact = { 0.0, 0.0, false };
  m_data.order = { 0.0, 0.0, false };
  m_data.pos_ent = { 0.0, 0.0, false };
  m_data.tdist_ent = { 0.0, 0.0, false };
  m_data.velocity = { 0.0, 0.0, false };
} /* reset_after_interval() */

NS_END(metrics, convergence, cosm);
