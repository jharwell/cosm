/**
 * \file population_dynamics_metrics_collector.cpp
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
#include "cosm/tv/metrics/population_dynamics_metrics_collector.hpp"

#include "cosm/tv/metrics/population_dynamics_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, tv, metrics);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
population_dynamics_metrics_collector::population_dynamics_metrics_collector(
    const std::string& ofname,
    uint interval)
    : base_metrics_collector(ofname, interval) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string> population_dynamics_metrics_collector::csv_header_cols(
    void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
      /* clang-format off */
    "int_avg_swarm_population",
    "cum_avg_swarm_population",
    "swarm_max_population",
    "int_avg_birth_queue_size",
    "cum_avg_birth_queue_size",
    "birth_mu",
    "int_avg_death_queue_size",
    "cum_avg_death_queue_size",
    "death_lambda",
    "int_avg_repair_queue_size",
    "cum_avg_repair_queue_size",
    "repair_lambda",
    "repair_mu",
      /* clang-format on */
  };
  merged.splice(merged.end(), cols);
  return merged;
} /* csv_header_cols() */

void population_dynamics_metrics_collector::reset(void) {
  base_metrics_collector::reset();
  reset_after_interval();
} /* reset() */

boost::optional<std::string> population_dynamics_metrics_collector::csv_line_build(
    void) {
  if (!((timestep() + 1) % interval() == 0)) {
    return boost::none;
  }
  std::string line;

  /* misc */
  line += csv_entry_intavg(m_interval.swarm_population);
  line += csv_entry_tsavg(m_cum.swarm_population);
  line += rcppsw::to_string(m_interval.swarm_max_population) + separator();

  /* birth queue */
  line += csv_entry_intavg(m_interval.birth_queue_size);
  line += csv_entry_tsavg(m_cum.birth_queue_size);
  line += rcppsw::to_string(m_interval.birth_mu) + separator();

  /* death queue */
  line += csv_entry_intavg(m_interval.death_queue_size);
  line += csv_entry_tsavg(m_cum.death_queue_size);
  line += rcppsw::to_string(m_interval.death_lambda) + separator();

  /* repair queue */
  line += csv_entry_intavg(m_interval.repair_queue_size);
  line += csv_entry_tsavg(m_cum.repair_queue_size);
  line += rcppsw::to_string(m_interval.repair_lambda) + separator();
  line += rcppsw::to_string(m_interval.repair_mu) + separator();

  return boost::make_optional(line);
} /* csv_line_build() */

void population_dynamics_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const population_dynamics_metrics&>(metrics);

  /* misc */
  m_interval.swarm_population += m.swarm_population();
  m_cum.swarm_population += m.swarm_population();
  m_interval.swarm_max_population = m.swarm_max_population();
  m_cum.swarm_max_population = m.swarm_max_population();

  /* birth queue */
  auto birth = m.birth_queue_status();
  m_interval.birth_queue_size += birth.size;
  m_interval.birth_mu = birth.mu;
  m_cum.birth_queue_size += birth.size;
  m_cum.birth_mu = birth.mu;

  /* death queue */
  auto death = m.death_queue_status();
  m_interval.death_queue_size += death.size;
  m_interval.death_lambda = death.lambda;
  m_cum.death_queue_size += death.size;
  m_cum.death_lambda = death.lambda;

  /* repair queue */
  auto repair = m.repair_queue_status();
  m_interval.repair_queue_size += repair.size;
  m_interval.repair_lambda = repair.lambda;
  m_interval.repair_mu = repair.mu;
  m_cum.repair_lambda = repair.lambda;
  m_cum.repair_mu = repair.mu;
} /* collect() */

void population_dynamics_metrics_collector::reset_after_interval(void) {
  m_interval.swarm_population = 0;
  m_interval.swarm_max_population = 0;
  m_interval.birth_queue_size = 0;
  m_interval.death_queue_size = 0;
  m_interval.repair_queue_size = 0;
  m_interval.birth_mu = 0.0;
  m_interval.death_lambda = 0;
  m_interval.repair_mu = 0;
  m_interval.repair_lambda = 0;
} /* reset_after_interval() */

NS_END(metrics, tv, cosm);