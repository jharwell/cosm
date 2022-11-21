/**
 * \file wander_random_thresh.hpp
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

#include "cosm/spatial/strategy/nest/acq/random_thresh.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::nest::acq {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class wander_random_thresh
 * \ingroup spatial strategy nest acq
 *
 * \brief Strategy for robot motion after it enters the nest, but before it
 * formally acquires it. Robots choose a random point between where the robot
 * enters the nest and the center of the center to treat as the nest center as
 * they phototaxis towards the center. As they phototaxis towards this point,
 * they also wander (needed for controllers with memory) and avoid collision.
 */

class wander_random_thresh : public cssnest::acq::random_thresh {
 public:
  wander_random_thresh(const cssnest::config::acq_config* config,
                       const csfsm::fsm_params* params,
                       rmath::rng* rng);

  /* Not move/copy constructable/assignable by default */
  wander_random_thresh(const wander_random_thresh&) = delete;
  wander_random_thresh& operator=(const wander_random_thresh&) = delete;
  wander_random_thresh(wander_random_thresh&&) = delete;
  wander_random_thresh& operator=(wander_random_thresh&&) = delete;

  /* taskable overrides */
  void task_execute(void) override final;

  /* strategy metrics */
  const cssnest::acq::base_acq* nest_acq_strategy(void) const override {
    return this;
  }

  std::unique_ptr<base_acq> clone(void) const override {
    csfsm::fsm_params params {
      saa(),
      inta_tracker(),
      nz_tracker()
    };
    return std::make_unique<wander_random_thresh>(config(), &params, rng());
  }
};


} /* namespace cosm::spatial::strategy::nest::acq */
