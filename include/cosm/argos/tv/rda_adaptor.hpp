/**
 * \file rda_adaptor.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"

#include "cosm/pal/pal.hpp"
#include "cosm/tv/switchable_tv_generator.hpp"
#include "cosm/tv/robot_dynamics_applicator.hpp"
#include "cosm/cosm.hpp"
#include "cosm/hal/robot.hpp"
#include "cosm/pal/argos/swarm_manager_adaptor.hpp"
#include "cosm/pal/argos/swarm_iterator.hpp"
#include "cosm/controller/block_carrying_controller.hpp"
#include "cosm/controller/irv_recipient_controller.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, argos, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class rda_adaptor
 * \ingroup argos tv
 *
 * \brief Adapts \ref ctv::robot_dynamics_applicator to work within the ARGoS
 * simulator.
 */
template<typename TController>
class rda_adaptor final : public rer::client<rda_adaptor<TController>>,
                                public ctv::robot_dynamics_applicator {
 public:
  static_assert(std::is_base_of<ccontroller::irv_recipient_controller, TController>::value,
                "TControllerType not derived from irv_recipient_controller");
  static_assert(std::is_base_of<ccontroller::block_carrying_controller,
                TController>::value,
                "TControllerType not derived from block_carrying_controller");

  rda_adaptor(
      const ctv::config::robot_dynamics_applicator_config* config,
      const cpargos::swarm_manager_adaptor* const sm)
      : ER_CLIENT_INIT("cosm.pal.argos.rda_adaptor"),
        robot_dynamics_applicator(config),
        mc_sm(sm) {}

  rda_adaptor(const rda_adaptor&) = delete;
  const rda_adaptor& operator=(const rda_adaptor&) = delete;

  double avg_motion_throttle(void) const override {
    double accum = 0.0;
    auto cb = [&](auto& controller) {
      accum += controller->applied_movement_throttle();
    };

    cpargos::swarm_iterator::controllers<TController,
                                            cpal::iteration_order::ekSTATIC>(
                                                mc_sm, cb, cpal::kRobotType);
    auto n_robots = mc_sm->GetSpace().GetEntitiesByType(cpal::kRobotType).size();
    if (0 == n_robots) {
      return 0.0;
    } else {
      return accum / n_robots;
    }
  }

  void update(void) override {
    if (!motion_throttling_enabled() && !bc_throttling_enabled()) {
      return;
    }
    rtypes::timestep t(mc_sm->GetSpace().GetSimulationClock());

    auto cb = [&](auto& controller) {
      if (auto* mt = motion_throttler(controller->entity_id())) {
        mt->update(t);
      }

      if (auto* bct = bc_throttler(controller->entity_id())) {
        bct->toggle(controller->is_carrying_block());
        bct->update(t);
      }
    };

    cpargos::swarm_iterator::controllers<TController,
                                            cpal::iteration_order::ekSTATIC>(
                                                mc_sm, cb, cpal::kRobotType);
  }

 private:
  /* clang-format off */
  const cpargos::swarm_manager_adaptor* const mc_sm;
  /* clang-format on */
};

NS_END(tv, argos, cosm);

