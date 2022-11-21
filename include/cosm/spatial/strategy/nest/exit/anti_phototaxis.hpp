/**
 * \file anti_phototaxis.hpp
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
#include "cosm/cosm.hpp"
#include "cosm/spatial/strategy/nest/exit/base_exit.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::nest::exit {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class anti_phototaxis
 * \ingroup spatial strategy nest exit
 *
 * \brief Strategy for robot motion after has completed a task in the nest, and
 * needs to leave it. Robots perform anti-phototaxis until they exit the
 * nest. Robots still avoid collisions while phototaxiing.
 */
class anti_phototaxis : public rer::client<anti_phototaxis>,
                        public cssnest::exit::base_exit {
 public:
  anti_phototaxis(const cssnest::config::exit_config* config,
                  const csfsm::fsm_params* params,
                  rmath::rng* rng);

  /* Not move/copy constructable/assignable by default */
  anti_phototaxis(const anti_phototaxis&) = delete;
  anti_phototaxis& operator=(const anti_phototaxis&) = delete;
  anti_phototaxis(anti_phototaxis&&) = delete;
  anti_phototaxis& operator=(anti_phototaxis&&) = delete;

  /* taskable overrides */
  void task_start(cta::taskable_argument*) override final;
  void task_reset(void) override final {
    m_task_running = false;
  }
  bool task_running(void) const override final { return m_task_running; }
  bool task_finished(void) const override final { return !m_task_running; }
  void task_execute(void) override final;

  std::unique_ptr<base_exit> clone(void) const override {
    csfsm::fsm_params params {
      saa(),
      inta_tracker(),
      nz_tracker()
    };
    return std::make_unique<anti_phototaxis>(config(), &params, rng());
  }

 private:
  /* clang-format off */
  bool m_task_running{false};
  /* clang-format on */
};


} /* namespace cosm::spatial::strategy::nest::exit */
