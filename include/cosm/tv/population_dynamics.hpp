/**
 * \file swarm_population.hpp
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

#ifndef INCLUDE_COSM_TV_POPULATION_DYNAMICS_HPP_
#define INCLUDE_COSM_TV_POPULATION_DYNAMICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <algorithm>
#include <queue>
#include <utility>
#include <vector>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/rcppsw.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/types/type_uuid.hpp"
#include "rcppsw/ds/poisson_queue.hpp"

#include "cosm/tv/config/population_dynamics_config.hpp"
#include "cosm/tv/metrics/population_dynamics_metrics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class population_dynamics
 * \ingroup tv
 *
 * \brief Orchestrates all applications of temporal variance all robots in the
 * swarm as it relates to existence within the swarm. This includes:
 *
 * - Creating, initializing, and adding new robots to the swarm over time
 *   according to a pure birth process.
 *
 * - Permanently removing robots from the swarm over time according to a pure
 *   death process.
 *
 * - Temporarily removing robots from the swarm and then restoring them later,
 *   simulating a malfunction-repair cycle.
 *
 * This class does not apply swarm population dynamics directly, as that
 * possibly requires platform-specific knowledge. Instead it houses the data for
 * all possible types of dynamics, and specifies the interface for application,
 * which must be implemented by derived classes.
 *
 * It also does not track the swarm size directly, as that may also require
 * platform-specific knowledge.
 */
class population_dynamics : public rer::client<population_dynamics>,
                            public metrics::population_dynamics_metrics {
 public:
  struct op_result {
    rtypes::type_uuid id;
    size_t pop_size;
  };

  population_dynamics(const config::population_dynamics_config* config,
                      size_t current_pop,
                      rmath::rng* rng);
  ~population_dynamics(void) override = default;

  population_dynamics(const population_dynamics&) = delete;
  const population_dynamics& operator=(const population_dynamics&) = delete;

  /**
   * \brief Update the state of all processes. Should be called once per
   * timestep.
   */
  void update(const rtypes::timestep& t);

  /* population dynamics metrics overrides */
  queue_status birth_queue_status(void) const override RCSW_PURE;
  queue_status death_queue_status(void) const override RCSW_PURE;
  queue_status repair_queue_status(void) const override RCSW_PURE;
  size_t swarm_max_population(void) const override { return mc_config.max_size; }
  size_t swarm_population(void) const override { return m_current_pop; }
  void reset_metrics(void) override final {
    m_birth.reset_metrics();
    m_death.reset_metrics();
    m_repair.reset_metrics();
  }
 protected:
  bool already_killed(const rtypes::type_uuid& id) {
    return m_death.contains(id);
  }

  /**
   * \brief Kill a random robot within the swarm.
   *
   * \return The ID of the robot that was added (\ref rtypes::constants::kNoUUID
   * if the robot could not be killed), along with the current swarm size.
   */
  virtual op_result robot_kill(void) = 0;

  /**
   * \brief Add a new robot to the swarm with the specified ID.
   *
   * \param max_pop Maximum swarm population (-1 indicates no limit)
   * \param id The ID of the robot to add.
   *
   * \return The ID of the robot that was added (\ref rtypes::constants::kNoUUID
   * if the robot could not be added), along with the current swarm size.
   */
  virtual op_result robot_add(int max_pop, const rtypes::type_uuid& id) = 0;

  /**
   * \brief Temporarily remove a robot from simulation/the swarm.
   *
   * \return The ID of the robot that was temporarily removed (\ref
   * rtypes::constants::kNoUUID if no robot could be removed), along with the
   * current swarm size.
   */
  virtual op_result robot_malfunction(void) = 0;

  /**
   * \brief Restore a the specified robot which had been temporarily removed
   * from the swarm/simulation.
   *
   * \return The ID of the robot that was restored (\ref
   * rtypes::constants::kNoUUID if no robot could be restored), along with the
   * current swarm size.
   */
  virtual op_result robot_repair(const rtypes::type_uuid& id) = 0;

 private:

  void death_dynamics(const rtypes::timestep& t);
  void birth_dynamics(const rtypes::timestep& t);
  void malfunction_dynamics(const rtypes::timestep& t);
  void repair_dynamics(const rtypes::timestep& t);

  /* clang-format off */
  const config::population_dynamics_config mc_config;

  size_t                                   m_current_pop;
  rtypes::timestep                         m_timestep{0};
  rmath::rng*                              m_rng;

  rds::poisson_queue<rtypes::type_uuid>    m_birth;
  rds::poisson_queue<rtypes::type_uuid>    m_death;
  rds::poisson_queue<rtypes::type_uuid>    m_repair;
  /* clang-format on */
};

NS_END(tv, cosm);

#endif /* INCLUDE_COSM_TV_POPULATION_DYNAMICS_HPP_ */
