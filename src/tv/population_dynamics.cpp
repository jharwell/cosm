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
    size_t max_pop,
    rmath::rng* rng)
    : ER_CLIENT_INIT("cosm.tv.population_dynamics"),
      mc_config(*config),
      mc_max_pop(max_pop),
      m_current_pop(current_pop),
      m_rng(rng) {}

/*******************************************************************************
 * Metrics
 ******************************************************************************/
population_dynamics::queue_status population_dynamics::death_queue_status(void) const {
  return queue_status {
    .size_change = m_timestep == m_last_death,
        .size = m_kills.size(),
        .lambda = mc_config.death_lambda,
        .mu = std::numeric_limits<double>::infinity()
        };
} /* death_queue_status() */

population_dynamics::queue_status population_dynamics::birth_queue_status(void) const {
  return queue_status {
    .size_change = m_timestep == m_last_birth,
        .size = mc_max_pop - m_current_pop,
        .lambda = std::numeric_limits<double>::infinity(),
        .mu = mc_config.birth_mu
        };
} /* birth_queue_status() */

population_dynamics::queue_status population_dynamics::repair_queue_status(void) const {
  return queue_status {
    .size_change = m_timestep == m_last_repair,
        .size = m_repairing.size(),
        .lambda = mc_config.repair_lambda,
        .mu = mc_config.repair_mu
        };
} /* repair_queue_status() */



/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void population_dynamics::update(const rtypes::timestep& t) {
  m_timestep = t;
  death_dynamics(t);
  birth_dynamics(t);
  malfunction_dynamics(t);
  repair_dynamics(t);
} /* update() */

void population_dynamics::death_dynamics(const rtypes::timestep& t) {
  if (m_last_death + t >= m_rng->exponential(mc_config.death_lambda)) {
    auto res = robot_kill();
    if (rtypes::constants::kNoUUID != res.id) {
      ER_ASSERT(res.pop_size == m_current_pop - 1,
                "Unexpected population change: %zu != %zu",
                res.pop_size,
                m_current_pop - 1);
      ER_INFO("Killed robot with ID=%d,pop_size=%zu", res.id.v(), res.pop_size);

      m_kills.push_back(res.id);
      m_last_death = t;
      m_current_pop = res.pop_size;
    }
  }
} /* death_dynamics() */

void population_dynamics::birth_dynamics(const rtypes::timestep& t) {
  if (m_last_birth + t >= m_rng->exponential(mc_config.birth_mu)) {
    /*
     * Use max population size as the starting point for generating robot IDs
     * to guarantee uniqueness.
     */
    auto next = rtypes::type_uuid(mc_max_pop + m_n_births);
    auto res = robot_add(mc_max_pop, next);
    if (rtypes::constants::kNoUUID != res.id) {
      ER_ASSERT(res.pop_size == m_current_pop + 1,
                "Unexpected population change: %zu != %zu",
                res.pop_size,
                m_current_pop + 1);
      ER_INFO("Added new robot with ID=%d,pop_size=%zu,max_size=%zu",
              res.id.v(),
              res.pop_size,
              mc_max_pop);

      ++m_n_births;
      m_last_birth = t;
      m_current_pop = res.pop_size;
    }
  }
} /* birth_dynamics() */

void population_dynamics::malfunction_dynamics(const rtypes::timestep& t) {
  if (m_last_malfunction + t >=
      m_rng->exponential(mc_config.repair_lambda)) {
    auto res = robot_malfunction();
    if (rtypes::constants::kNoUUID != res.id) {
      ER_ASSERT(res.pop_size == m_current_pop,
                "Unexpected population change: %zu != %zu",
                res.pop_size,
                m_current_pop);
      ER_INFO("Added robot with ID=%d to repair queue", res.id.v());

      m_repairing.push(res.id);
      ++m_total_malfunctions;
      m_last_malfunction = t;
    }
  }
} /* malfunction_dynamics() */

void population_dynamics::repair_dynamics(const rtypes::timestep& t) {
  if (m_last_repair + t >= m_rng->exponential(mc_config.repair_mu) &&
      !m_repairing.empty()) {
    rtypes::type_uuid id = m_repairing.front();
    auto res = robot_repair(id);
    ER_ASSERT(res.pop_size == m_current_pop,
              "Unexpected population change: %zu != %zu",
              res.pop_size,
              m_current_pop);
    ER_ASSERT(res.id == id,
              "Repaired robot uuid (%d) != robot at front of repair queue (%d)",
              res.id.v(),
              id.v());
    ER_INFO("Removed robot with ID=%d from repair queue", res.id.v());

    m_repairing.pop();
    ++m_total_repairs;
    m_last_repair = t;
  }
} /* repair_dynamics() */

NS_END(tv, cosm);
