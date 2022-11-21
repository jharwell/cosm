/**
 * \file stoch_fov.hpp
 *
 * \copyright 2022 SIFT LLC, All rights reserved.
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
#include "cosm/spatial/strategy/flocking/base_flocking.hpp"
#include "cosm/flocking/interaction_probability.hpp"
#include "cosm/apf2D/boid_vector.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::flocking {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class stoch_fov
 * \ingroup spatial strategy flocking
 *
 * \brief Strategy for robot flocking implementing the stochastic Field Of View
 * (FOV) approach from \cite FLOCK:Bagarti2018-stochfov.
 *
 * Agents stochastically interact with a SINGLE agent chosen from other agents
 * within its FOV according to \ref cflocking::interaction_probability.  Agents
 * do not perform collision avoidance.
 */
class stoch_fov : public rer::client<stoch_fov>,
                  public cssflocking::base_flocking {
 public:
  stoch_fov(const cssflocking::config::flocking_config* config,
            const csfsm::fsm_params* params,
            rmath::rng* rng);

  /* Not move/copy constructable/assignable by default */
  stoch_fov(const stoch_fov&) = delete;
  stoch_fov& operator=(const stoch_fov&) = delete;
  stoch_fov(stoch_fov&&) = delete;
  stoch_fov& operator=(stoch_fov&&) = delete;

  void set_inputs(const capf2D::boid_vectorro& boids) { m_boids = boids; }

  /* strategy metrics */

  /* taskable overrides */
  void task_start(cta::taskable_argument*) override final {};
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
  bool                               m_task_running{false};
  capf2D::boid_vectorro              m_boids{};
  cflocking::interaction_probability m_prob;
  /* clang-format on */
};

} /* namespace cosm::spatial::strategy::flocking */
