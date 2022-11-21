/**
 * \file stoch_fov.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <vector>

#include "cosm/cosm.hpp"
#include "cosm/flocking/base_flocking.hpp"
#include "cosm/flocking/interaction_probability.hpp"
#include "cosm/apf2D/boid_vector.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::flocking {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class stoch_fov
 * \ingroup flocking
 *
 * \brief Strategy for robot flocking implementing the stochastic Field Of View
 * (FOV) approach from \cite FLOCK:Bagarti2018-stochfov.
 *
 * Agents stochastically interact with a SINGLE agent chosen from other agents
 * within its FOV according to \ref cflocking::interaction_probability.  Agents
 * do not perform collision avoidance.
 */
class stoch_fov : public rer::client<stoch_fov>,
                  public cflocking::base_flocking {
 public:
  stoch_fov(const cflocking::config::flocking_config* config,
            const csfsm::fsm_params* params,
            rmath::rng* rng);

  /* Not move/copy constructable/assignable by default */
  stoch_fov(const stoch_fov&) = delete;
  stoch_fov& operator=(const stoch_fov&) = delete;
  stoch_fov(stoch_fov&&) = delete;
  stoch_fov& operator=(stoch_fov&&) = delete;

  void set_inputs(const std::vector<ckin::odometry>& odom) { m_odom = odom; }

  /* strategy metrics */

  /* taskable overrides */
  void task_start(cta::taskable_argument*) override final;
  void task_reset(void) override final;
  bool task_running(void) const override final { return m_task_running; }
  bool task_finished(void) const override final { return !m_task_running; }
  void task_execute(void) override final;

  std::unique_ptr<base_flocking> clone(void) const override {
    csfsm::fsm_params params {
      saa(),
      inta_tracker(),
      nz_tracker()
    };
    return std::make_unique<stoch_fov>(config(), &params, rng());
  }

 private:
  /* clang-format off */
  size_t                             m_init_count{true};
  bool                               m_task_running{false};
  std::vector<ckin::odometry>        m_odom{};
  cflocking::interaction_probability m_prob;
  /* clang-format on */
};

} /* namespace cosm::flocking */
