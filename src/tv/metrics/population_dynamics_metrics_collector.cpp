/**
 * \file population_dynamics_metrics_collector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/tv/metrics/population_dynamics_metrics_collector.hpp"

#include "cosm/tv/metrics/population_dynamics_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::tv::metrics {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
population_dynamics_metrics_collector::population_dynamics_metrics_collector(
    std::unique_ptr<rmetrics::base_sink> sink)
    : base_collector(std::move(sink)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void population_dynamics_metrics_collector::collect(
    const rmetrics::base_metrics& metrics) {
  auto& m = dynamic_cast<const population_dynamics_metrics&>(metrics);

  /* population */
  m_data.interval.total_population += m.swarm_total_population();
  m_data.interval.active_population += m.swarm_active_population();
  m_data.interval.max_population = m.swarm_max_population();

  m_data.cum.total_population += m.swarm_total_population();
  m_data.cum.active_population += m.swarm_active_population();

  /* birth queue */
  auto birth = m.birth_queue_status();
  m_data.interval.n_births += birth.dequeue.count;
  m_data.interval.birth_interval += birth.dequeue.interval_accum.v();
  m_data.interval.birth_mu = birth.mu;

  m_data.cum.n_births += birth.dequeue.count;
  m_data.cum.birth_interval += birth.dequeue.interval_accum.v();
  m_data.cum.birth_mu = birth.mu;

  /* death queue */
  auto death = m.death_queue_status();
  m_data.interval.n_deaths += death.enqueue.count;
  m_data.interval.death_interval += death.enqueue.interval_accum.v();
  m_data.interval.death_lambda = death.lambda;

  m_data.cum.n_deaths += death.enqueue.count;
  m_data.cum.death_interval += death.enqueue.interval_accum.v();
  m_data.cum.death_lambda = death.lambda;

  /* repair queue */
  auto repair = m.repair_queue_status();
  m_data.interval.repair_queue_size += repair.size;
  m_data.cum.repair_queue_size += repair.size;

  m_data.interval.n_malfunctions += repair.enqueue.count;
  m_data.interval.malfunction_interval += repair.enqueue.interval_accum.v();
  m_data.interval.malfunction_lambda = repair.lambda;

  m_data.interval.n_repairs += repair.dequeue.count;
  m_data.interval.repair_interval += repair.dequeue.interval_accum.v();
  m_data.interval.repair_mu = repair.mu;

  m_data.cum.n_malfunctions += repair.enqueue.count;
  m_data.cum.malfunction_interval += repair.enqueue.interval_accum.v();
  m_data.cum.malfunction_lambda = repair.lambda;

  m_data.cum.n_repairs += repair.dequeue.count;
  m_data.cum.repair_interval += repair.dequeue.interval_accum.v();
  m_data.cum.repair_mu = repair.mu;

  m_data.cum.malfunction_lambda = repair.lambda;
  m_data.cum.repair_mu = repair.mu;
} /* collect() */

void population_dynamics_metrics_collector::reset_after_interval(void) {
  m_data.interval.total_population = 0;
  m_data.interval.active_population = 0;
  m_data.interval.max_population = 0;

  m_data.interval.n_births = 0;
  m_data.interval.birth_interval = 0;
  m_data.interval.birth_mu = 0;

  m_data.interval.n_deaths = 0;
  m_data.interval.death_interval = 0;
  m_data.interval.death_lambda = 0;

  m_data.interval.repair_queue_size = 0;
  m_data.interval.n_malfunctions = 0;
  m_data.interval.malfunction_interval = 0;
  m_data.interval.malfunction_lambda = 0;

  m_data.interval.n_repairs = 0;
  m_data.interval.repair_interval = 0;
  m_data.interval.repair_mu = 0;
} /* reset_after_interval() */

} /* namespace cosm::tv::metrics */
