/**
 * \file wander.hpp
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
#include <memory>
#include "cosm/cosm.hpp"
#include "cosm/spatial/strategy/nest/exit/base_exit.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy, nest, exit);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class wander
 * \ingroup spatial strategy nest exit
 *
 * \brief Strategy for robot motion after has completed a task in the nest, and
 * needs to leave it. Robots wander until they happen to exit the nest or get
 * pushed out via collision avoidance.
 */
class wander : public rer::client<wander>,
               public cssnest::exit::base_exit {
 public:
  wander(const cssnest::config::exit_config* config,
         const csfsm::fsm_params* params,
         rmath::rng* rng);

  /* Not move/copy constructable/assignable by default */
  wander(const wander&) = delete;
  wander& operator=(const wander&) = delete;
  wander(wander&&) = delete;
  wander& operator=(wander&&) = delete;

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
    return std::make_unique<wander>(config(), &params, rng());
  }

 private:
  /* clang-format off */
  bool m_task_running{false};
  /* clang-format on */
};


NS_END(exit, nest, strategy, spatial, cosm);
