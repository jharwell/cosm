/**
 * \file population_dynamics.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "cosm/tv/population_dynamics.hpp"

#include "rcppsw/math/config/rng_config.hpp"

#include "cosm/tv/config/population_dynamics_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, tv);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
population_dynamics::population_dynamics(
    const config::population_dynamics_config* const config,
    size_t current_pop,
    rmath::rng* rng)
    : ER_CLIENT_INIT("cosm.tv.population_dynamics"),
      mc_config(*config),
      m_current_pop(current_pop),
      m_rng(rng),
      m_birth(std::numeric_limits<double>::infinity(), config->birth_mu, rng),
      m_death(config->death_lambda, std::numeric_limits<double>::infinity(), rng),
      m_repair(config->malfunction_lambda, config->repair_mu, rng) {}

/*******************************************************************************
 * Metrics
 ******************************************************************************/
population_dynamics::queue_status population_dynamics::death_queue_status(
    void) const {
  auto ed = m_death.enqueue_data();
  auto dd = m_death.dequeue_data();
  queue_op_status enqueue = {.count = ed.count,
                             .interval_accum = ed.interval_accum};
  queue_op_status dequeue = {.count = dd.count,
                             .interval_accum = dd.interval_accum};
  return queue_status{.size = m_death.size(),
                      .lambda = m_death.lambda(),
                      .mu = m_death.mu(),
                      .enqueue = enqueue,
                      .dequeue = dequeue};
} /* death_queue_status() */

population_dynamics::queue_status population_dynamics::birth_queue_status(
    void) const {
  auto ed = m_birth.enqueue_data();
  auto dd = m_birth.dequeue_data();
  queue_op_status enqueue = {.count = ed.count,
                             .interval_accum = ed.interval_accum};
  queue_op_status dequeue = {.count = dd.count,
                             .interval_accum = dd.interval_accum};
  return queue_status{.size = std::numeric_limits<size_t>::infinity(),
                      .lambda = m_birth.lambda(),
                      .mu = m_birth.mu(),
                      .enqueue = enqueue,
                      .dequeue = dequeue};
} /* birth_queue_status() */

population_dynamics::queue_status population_dynamics::repair_queue_status(
    void) const {
  auto ed = m_repair.enqueue_data();
  auto dd = m_repair.dequeue_data();
  queue_op_status enqueue = {.count = ed.count,
                             .interval_accum = ed.interval_accum};
  queue_op_status dequeue = {.count = dd.count,
                             .interval_accum = dd.interval_accum};
  return queue_status{.size = m_repair.size(),
                      .lambda = m_repair.lambda(),
                      .mu = m_repair.mu(),
                      .enqueue = enqueue,
                      .dequeue = dequeue};
} /* repair_queue_status() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void population_dynamics::update(const rtypes::timestep& t) {
  m_timestep = t;

  m_death.reset_metrics();
  death_dynamics(t);

  m_birth.reset_metrics();
  birth_dynamics(t);

  m_repair.reset_metrics();
  malfunction_dynamics(t);
  repair_dynamics(t);
} /* update() */

void population_dynamics::death_dynamics(const rtypes::timestep& t) {
  if (m_death.enqueue_check(t)) {
    auto res = robot_kill();
    if (rtypes::constants::kNoUUID != res.id) {
      ER_ASSERT(res.pop_size == m_current_pop - 1,
                "Unexpected population change: %zu != %zu",
                res.pop_size,
                m_current_pop - 1);
      ER_INFO("Killed robot with ID=%d,pop_size=%zu", res.id.v(), res.pop_size);
      m_death.enqueue(res.id, t);
      m_current_pop = res.pop_size;
    }
  }
} /* death_dynamics() */

void population_dynamics::birth_dynamics(const rtypes::timestep& t) {
  if (m_birth.dequeue_check(t)) {
    /*
     * Use max population size as the starting point for generating robot IDs
     * to guarantee uniqueness.
     */
    auto next = rtypes::type_uuid(mc_config.max_size +
                                  m_birth.dequeue_data().total_count);
    auto res = robot_add(mc_config.max_size, next);
    if (rtypes::constants::kNoUUID != res.id) {
      ER_ASSERT(res.pop_size == m_current_pop + 1,
                "Unexpected population change: %zu != %zu",
                res.pop_size,
                m_current_pop + 1);
      ER_INFO("Added new robot with ID=%d,pop_size=%zu,max_size=%d",
              res.id.v(),
              res.pop_size,
              mc_config.max_size);

      m_birth.dequeue(t, true);
      m_current_pop = res.pop_size;
    }
  }
} /* birth_dynamics() */

void population_dynamics::malfunction_dynamics(const rtypes::timestep& t) {
  if (m_repair.enqueue_check(t)) {
    auto res = robot_malfunction();
    if (rtypes::constants::kNoUUID != res.id) {
      ER_ASSERT(res.pop_size == m_current_pop,
                "Unexpected population change: %zu != %zu",
                res.pop_size,
                m_current_pop);
      ER_INFO("Added robot with ID=%d to repair queue", res.id.v());
      m_repair.enqueue(res.id, t);
    }
  }
} /* malfunction_dynamics() */

void population_dynamics::repair_dynamics(const rtypes::timestep& t) {
  if (m_repair.dequeue_check(t)) {
    auto id = m_repair.dequeue(t, false);
    auto res = robot_repair(*id);
    ER_ASSERT(res.pop_size == m_current_pop,
              "Unexpected population change: %zu != %zu",
              res.pop_size,
              m_current_pop);
    ER_ASSERT(res.id == id,
              "Repaired robot uuid (%d) != robot at front of repair queue (%d)",
              res.id.v(),
              id->v());
    ER_INFO("Removed robot with ID=%d from repair queue", res.id.v());
  }
} /* repair_dynamics() */

NS_END(tv, cosm);
