/**
 * \file wander_random_thresh.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_SPATIAL_STRATEGY_NEST_ACQ_WANDER_RANDOM_THRESH_HPP_
#define INCLUDE_COSM_SPATIAL_STRATEGY_NEST_ACQ_WANDER_RANDOM_THRESH_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "cosm/spatial/strategy/nest_acq/random_thresh.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy, nest_acq);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class wander_random_thresh
 * \ingroup spatial strategy nest_acq
 *
 * \brief Strategy for robot motion after it enters the nest, but before it
 * formally acquires it. Robots choose a random point between where the robot
 * enters the nest and the center of the center to treat as the nest center as
 * they phototaxis towards the center. As they phototaxis towards this point,
 * they also wander (needed for controllers with memory) and avoid collision.
 */

class wander_random_thresh : public csstrategy::nest_acq::random_thresh {
 public:
  wander_random_thresh(csubsystem::saa_subsystemQ3D* saa, rmath::rng* rng)
      : random_thresh(saa, rng) {}

  /* Not move/copy constructable/assignable by default */
  wander_random_thresh(const wander_random_thresh&) = delete;
  wander_random_thresh& operator=(const wander_random_thresh&) = delete;
  wander_random_thresh(wander_random_thresh&&) = delete;
  wander_random_thresh& operator=(wander_random_thresh&&) = delete;

  /* taskable overrides */
  void task_execute(void) override final;

  /* strategy metrics */
  const cssnest_acq::base_nest_acq* nest_acq_strategy(void) const override {
    return this;
  }

  std::unique_ptr<base_strategy> clone(void) const override {
    return std::make_unique<wander_random_thresh>(saa(), rng());
  }

 private:
  /* clang-format off */
  /* clang-format on */
};


NS_END(nest_acq3, strategy, spatial, cosm);

#endif /* INCLUDE_COSM_SPATIAL_STRATEGY_NEST_ACQ_WANDER_RANDOM_THRESH_HPP_ */
