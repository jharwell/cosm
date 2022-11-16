/**
 * \file stoch_fov.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include "cosm/cosm.hpp"
#include "cosm/spatial/strategy/flocking/base_flocking.hpp"

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
 * (FOV) approach from Bagarti2018. Agents stochastically interact with
 * a SINGLE agents chosen from other agents within its FOV according to
 * \ref cflocking::interaction_probability.
 *
 * Agents do NOT perform collision avoidance.
 */
class stoch_fov : public rer::client<stoch_fov>,
                       public cssnest::acq::base_acq {
 public:
  stoch_fov(const cssflocking::config::flocking_config* config,
         const csfsm::fsm_params* params,
         rmath::rng* rng);

  /* Not move/copy constructable/assignable by default */
  stoch_fov(const stoch_fov&) = delete;
  stoch_fov& operator=(const stoch_fov&) = delete;
  stoch_fov(stoch_fov&&) = delete;
  stoch_fov& operator=(stoch_fov&&) = delete;

  /* strategy metrics */
  const cssflocking::base_flocking* flocking_strategy(void) const override {
    return this;
  }

  /* taskable overrides */
  void task_start(cta::taskable_argument*) override final;
  void task_reset(void) override final {
    m_task_running = false;
  }
  bool task_running(void) const override final { return m_task_running; }
  bool task_finished(void) const override final { return !m_task_running; }
  void task_execute(void) override final;

  std::unique_ptr<base_acq> clone(void) const override {
    csfsm::fsm_params params {
      saa(),
      inta_tracker(),
      nz_tracker()
    };
    return std::make_unique<stoch_fov>(config(), &params, rng());
  }

 private:
  /* clang-format off */
  bool m_task_running{false};
  /* clang-format on */
};

} /* namespace cosm::spatial::strategy::flocking */
