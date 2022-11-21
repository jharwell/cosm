/**
 * \file wander.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include "cosm/cosm.hpp"
#include "cosm/spatial/strategy/nest/acq/base_acq.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::nest::acq {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class wander
 * \ingroup spatial strategy nest acq
 *
 * \brief Strategy for robot motion after it enters the nest, but before it
 * formally acquires it. Robots wander for a set # timesteps after entering the
 * nest so that the location the robot actually acquires is distributed around
 * the precise goal location to reduce congestion. This helps keep large
 * clusters of avoiding robots from forming near common goals as they all
 * attempt to acquire points VERY close to each other.
 *
 * Robots continue to phototaxis towards the light and avoid collisions while
 * wandering.
 */
class wander : public rer::client<wander>,
               public cssnest::acq::base_acq {
 public:
  wander(const cssnest::config::acq_config* config,
         const csfsm::fsm_params* params,
         rmath::rng* rng);

  /* Not move/copy constructable/assignable by default */
  wander(const wander&) = delete;
  wander& operator=(const wander&) = delete;
  wander(wander&&) = delete;
  wander& operator=(wander&&) = delete;

  /* strategy metrics */
  const cssnest::acq::base_acq* nest_acq_strategy(void) const override {
    return this;
  }

  /* taskable overrides */
  void task_start(cta::taskable_argument*) override final;
  void task_reset(void) override final {
    m_task_running = false;
    m_steps = rtypes::timestep(0);
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
    return std::make_unique<wander>(config(), &params, rng());
  }

 private:
  /* clang-format off */
  bool             m_task_running{false};
  rtypes::timestep m_steps{0};
  /* clang-format on */
};


} /* namespace cosm::spatial::strategy::nest::acq */
