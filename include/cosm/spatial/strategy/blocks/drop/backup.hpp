/**
 * \file backup.hpp
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
#include "cosm/spatial/strategy/blocks/drop/base_drop.hpp"
#include "cosm/kin/odometry.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy, blocks, drop);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class backup
 * \ingroup spatial strategy blocks drop
 *
 * \brief Strategy for robot motion after it has acquired the location it wants
 * to drop. Robots backup for a set # timesteps, and then continue on their
 * way. This is useful if robots are pushing objects rather than carrying them
 * to get them to "drop".
 *
 * Robots do not avoid collisions while backing up (steering forces are
 * disabled).
 */
class backup : public rer::client<backup>,
               public cssblocks::drop::base_drop {
 public:
  backup(const csfsm::fsm_params* params,
         const cssblocks::config::drop_config* config,
         rmath::rng* rng);

  /* Not move/copy constructable/assignable by default */
  backup(const backup&) = delete;
  backup& operator=(const backup&) = delete;
  backup(backup&&) = delete;
  backup& operator=(backup&&) = delete;

  /* taskable overrides */
  void task_start(cta::taskable_argument*) override final;
  void task_reset(void) override final;
  bool task_running(void) const override final { return m_task_running; }
  bool task_finished(void) const override final { return !m_task_running; }
  void task_execute(void) override final;

  std::unique_ptr<base_drop> clone(void) const override {
    csfsm::fsm_params params {
      saa(),
      inta_tracker(),
      nz_tracker()
    };
    return std::make_unique<backup>(&params, config(), rng());
  }

 private:
  /* clang-format off */
  bool             m_task_running{false};
  rtypes::timestep m_duration;
  rtypes::timestep m_steps{rtypes::timestep(0)};
  ckin::odometry   m_odom_start{};
  /* clang-format on */
};


NS_END(drop, blocks, strategy, spatial, cosm);
