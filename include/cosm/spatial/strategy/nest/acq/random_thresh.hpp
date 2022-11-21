/**
 * \file random_thresh.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>
#include <memory>

#include "rcppsw/spatial/euclidean_dist.hpp"

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
 * \class random_thresh
 * \ingroup spatial strategy nest acq
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

class random_thresh : public cssnest::acq::base_acq {
 public:
  random_thresh(const cssnest::config::acq_config* config,
                const csfsm::fsm_params* params,
                rmath::rng* rng);

  /* Not move/copy constructable/assignable by default */
  random_thresh(const random_thresh&) = delete;
  random_thresh& operator=(const random_thresh&) = delete;
  random_thresh(random_thresh&&) = delete;
  random_thresh& operator=(random_thresh&&) = delete;

  /* strategy metrics */
  const cssnest::acq::base_acq* nest_acq_strategy(void) const override {
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

  std::unique_ptr<base_acq> clone(void) const override {
    csfsm::fsm_params params {
      saa(),
      inta_tracker(),
      nz_tracker()
    };
    return std::make_unique<random_thresh>(config(), &params, rng());
  }

  boost::optional<rspatial::euclidean_dist> thresh(void) const {
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
  rspatial::euclidean_dist m_thresh{rspatial::euclidean_dist(0)};
  /* clang-format on */
};


} /* namespace cosm::spatial::strategy::nest::acq */
