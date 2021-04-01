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
    const std::string& ofname_stem,
    const rtypes::timestep& interval)
    : base_metrics_collector(ofname_stem,
                             interval,
                             rmetrics::output_mode::ekAPPEND) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::list<std::string>
population_dynamics_metrics_collector::csv_header_cols(void) const {
  auto merged = dflt_csv_header_cols();
  auto cols = std::list<std::string>{
    /* clang-format off */
    "int_avg_total_population",
    "int_avg_active_population",
    "cum_avg_total_population",
    "cum_avg_active_population",
    "max_population",

    "int_avg_birth_rate",
    "int_avg_birth_interval",
    "cum_avg_birth_rate",
    "cum_avg_birth_interval",
    "birth_mu",

    "int_avg_death_rate",
    "int_avg_death_interval",
    "cum_avg_death_rate",
    "cum_avg_death_interval",
    "death_lambda",

    "int_avg_repair_queue_size",
    "cum_avg_repair_queue_size",

    "int_avg_malfunction_rate",
    "int_avg_malfunction_interval",
    "cum_avg_malfunction_rate",
    "cum_avg_malfunction_interval",
    "malfunction_lambda",

    "int_avg_repair_rate",
    "int_avg_repair_interval",
    "cum_avg_repair_rate",
    "cum_avg_repair_interval",
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

boost::optional<std::string>
population_dynamics_metrics_collector::csv_line_build(void) {
  if (!(timestep() % interval() == 0UL)) {
    return boost::none;
  }
  std::string line;

  /* population */
  line += csv_entry_intavg(m_interval.total_population);
  line += csv_entry_intavg(m_interval.active_population);
  line += csv_entry_tsavg(m_cum.total_population);
  line += csv_entry_tsavg(m_cum.active_population);
  line += rcppsw::to_string(m_interval.max_population) + separator();

  /* birth queue */
  line += csv_entry_intavg(m_interval.n_births);
  line += csv_entry_domavg(m_interval.birth_interval, m_interval.n_births);

  line += csv_entry_tsavg(m_cum.n_births);
  line += csv_entry_domavg(m_cum.birth_interval, m_cum.n_births);

  line += rcppsw::to_string(m_interval.birth_mu) + separator();

  /* death queue */
  line += csv_entry_intavg(m_interval.n_deaths.load());
  line += csv_entry_domavg(m_interval.death_interval.load(),
                           m_interval.n_deaths.load());

  line += csv_entry_tsavg(m_cum.n_deaths.load());
  line += csv_entry_domavg(m_cum.death_interval.load(), m_cum.n_deaths.load());
  line += rcppsw::to_string(m_interval.death_lambda) + separator();

  /* repair queue */
  line += csv_entry_intavg(m_interval.repair_queue_size);
  line += csv_entry_tsavg(m_cum.repair_queue_size);

  /* repair queue malfunctions */
  line += csv_entry_intavg(m_interval.n_malfunctions);
  line += csv_entry_domavg(m_interval.malfunction_interval,
                           m_interval.n_malfunctions);

  line += csv_entry_tsavg(m_cum.n_malfunctions);
  line += csv_entry_domavg(m_cum.malfunction_interval, m_cum.n_malfunctions);
  line += rcppsw::to_string(m_interval.malfunction_lambda) + separator();

  /* repair queue repairs */
  line += csv_entry_intavg(m_interval.n_repairs);
  line += csv_entry_domavg(m_interval.repair_interval, m_interval.n_repairs);

  line += csv_entry_tsavg(m_cum.n_repairs);
  line += csv_entry_domavg(m_cum.repair_interval, m_cum.n_repairs);
  line += rcppsw::to_string(m_interval.repair_mu);

  return boost::make_optional(line);
} /* csv_line_build() */

void population_dynamics_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const population_dynamics_metrics&>(metrics);

  /* population */
  m_interval.total_population += m.swarm_total_population();
  m_interval.active_population += m.swarm_active_population();
  m_interval.max_population = m.swarm_max_population();

  m_cum.total_population += m.swarm_total_population();
  m_cum.active_population += m.swarm_active_population();

  /* birth queue */
  auto birth = m.birth_queue_status();
  m_interval.n_births += birth.dequeue.count;
  m_interval.birth_interval += birth.dequeue.interval_accum.v();
  m_interval.birth_mu = birth.mu;

  m_cum.n_births += birth.dequeue.count;
  m_cum.birth_interval += birth.dequeue.interval_accum.v();
  m_cum.birth_mu = birth.mu;

  /* death queue */
  auto death = m.death_queue_status();
  m_interval.n_deaths += death.enqueue.count;
  m_interval.death_interval += death.enqueue.interval_accum.v();
  m_interval.death_lambda = death.lambda;

  m_cum.n_deaths += death.enqueue.count;
  m_cum.death_interval += death.enqueue.interval_accum.v();
  m_cum.death_lambda = death.lambda;

  /* repair queue */
  auto repair = m.repair_queue_status();
  m_interval.repair_queue_size += repair.size;
  m_cum.repair_queue_size += repair.size;

  m_interval.n_malfunctions += repair.enqueue.count;
  m_interval.malfunction_interval += repair.enqueue.interval_accum.v();
  m_interval.malfunction_lambda = repair.lambda;

  m_interval.n_repairs += repair.dequeue.count;
  m_interval.repair_interval += repair.dequeue.interval_accum.v();
  m_interval.repair_mu = repair.mu;

  m_cum.n_malfunctions += repair.enqueue.count;
  m_cum.malfunction_interval += repair.enqueue.interval_accum.v();
  m_cum.malfunction_lambda = repair.lambda;

  m_cum.n_repairs += repair.dequeue.count;
  m_cum.repair_interval += repair.dequeue.interval_accum.v();
  m_cum.repair_mu = repair.mu;

  m_cum.malfunction_lambda = repair.lambda;
  m_cum.repair_mu = repair.mu;
} /* collect() */

void population_dynamics_metrics_collector::reset_after_interval(void) {
  m_interval.total_population = 0;
  m_interval.active_population = 0;
  m_interval.max_population = 0;

  m_interval.n_births = 0;
  m_interval.birth_interval = 0;
  m_interval.birth_mu = 0;

  m_interval.n_deaths = 0;
  m_interval.death_interval = 0;
  m_interval.death_lambda = 0;

  m_interval.repair_queue_size = 0;
  m_interval.n_malfunctions = 0;
  m_interval.malfunction_interval = 0;
  m_interval.malfunction_lambda = 0;

  m_interval.n_repairs = 0;
  m_interval.repair_interval = 0;
  m_interval.repair_mu = 0;
} /* reset_after_interval() */

NS_END(metrics, tv, cosm);
