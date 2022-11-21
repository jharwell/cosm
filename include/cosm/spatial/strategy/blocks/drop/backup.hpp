/**
 * \file backup.hpp
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
#include "cosm/spatial/strategy/blocks/drop/base_drop.hpp"
#include "cosm/kin/odometry.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial::strategy::blocks::drop {

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

  /* drop metrics */
  const cssblocks::drop::base_drop* block_drop_strategy(void) const override {
    return this;
  }

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


} /* namespace cosm::spatial::strategy::blocks::drop */
