/**
 * \file random_thresh.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>
#include <memory>

#include "rcppsw/types/spatial_dist.hpp"

#include "cosm/cosm.hpp"
#include "cosm/spatial/strategy/nest_acq/base_nest_acq.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy, nest_acq);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class random_thresh
 * \ingroup spatial strategy nest_acq
 *
 * \brief Strategy for robot motion after it enters the nest, but before it
 * formally acquires it. Robots choose a random point between where the robot
 * enters the nest and the center of the center to treat as the nest center as
 * they phototaxis towards the center. This allows for SOME collision avoidance
 * strategy when dropping stuff in the nest while also still being precisely
 * modelable (i.e., the average distance robot travels after entering the nest
 * before dropping its object is computable).
 *
 * Robots continue to avoid collisions while photoaxiing.
 *
 * Does NOT work well with controllers with memory.
 */

class random_thresh : public csstrategy::nest_acq::base_nest_acq {
 public:
  random_thresh(const csfsm::fsm_params* params, rmath::rng* rng)
      : base_nest_acq(params, rng) {}

  /* Not move/copy constructable/assignable by default */
  random_thresh(const random_thresh&) = delete;
  random_thresh& operator=(const random_thresh&) = delete;
  random_thresh(random_thresh&&) = delete;
  random_thresh& operator=(random_thresh&&) = delete;

  /* strategy metrics */
  const cssnest_acq::base_nest_acq* nest_acq_strategy(void) const override {
    return this;
  }

  /* taskable overrides */
  void task_start(cta::taskable_argument* arg) override final;
  void task_reset(void) override final {
    m_task_running = false;
  }
  bool task_running(void) const override final { return m_task_running; }
  bool task_finished(void) const override final { return !m_task_running; }
  void task_execute(void) override;

  std::unique_ptr<base_strategy> clone(void) const override {
    csfsm::fsm_params params {
      saa(),
      inta_tracker(),
      nz_tracker()
    };
    return std::make_unique<random_thresh>(&params, rng());
  }

  boost::optional<rtypes::spatial_dist> thresh(void) const {
    if (task_running()) {
      return boost::make_optional(m_thresh);
    } else {
      return boost::none;
    }
  }

 private:
  /* clang-format off */
  bool                 m_task_running{false};
  rmath::vector2d      m_nest_loc{};
  rtypes::spatial_dist m_thresh{rtypes::spatial_dist(0)};
  /* clang-format on */
};


NS_END(nest_acq, strategy, spatial, cosm);

