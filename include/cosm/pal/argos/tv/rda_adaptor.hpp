/**
 * \file rda_adaptor.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_PAL_TV_ARGOS_RDA_ADAPTOR_HPP_
#define INCLUDE_COSM_PAL_TV_ARGOS_RDA_ADAPTOR_HPP_

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
NS_START(cosm, pal, argos, tv);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class rda_adaptor
 * \ingroup pal argos tv
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

NS_END(tv, argos, pal, cosm);

#endif /* INCLUDE_COSM_PAL_TV_ARGOS_RDA_ADAPTOR_HPP_ */
