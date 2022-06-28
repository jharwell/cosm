/**
 * \file foraging_util_hfsm.hpp
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

#include "cosm/spatial/fsm/util_hfsm.hpp"

#include "cosm/spatial/strategy/nest/metrics/acq_metrics.hpp"
#include "cosm/spatial/strategy/blocks/metrics/drop_metrics.hpp"
#include "cosm/foraging/fsm/strategy_set.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, foraging, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class foraging_util_hfsm
 * \ingroup foraging fsm
 *
 * \brief A collection of base states/common functionality that foraging FSMs
 * which operate in strict 2D or quasi-3D can use if they choose (NOT full 3D
 * robots).
 *
 * This class cannot be instantiated on its own, as does not define an FSM
 * per-se.
 */
class foraging_util_hfsm : public csfsm::util_hfsm,
                           public cssnest::metrics::acq_metrics,
                           public cssblocks::metrics::drop_metrics,
                           public rer::client<foraging_util_hfsm> {
 public:
  foraging_util_hfsm(const csfsm::fsm_params* params,
                     cffsm::strategy_set strategies,
                     rmath::rng* rng,
                     uint8_t max_states);

  ~foraging_util_hfsm(void) override = default;

  foraging_util_hfsm(const foraging_util_hfsm&) = delete;
  foraging_util_hfsm& operator=(const foraging_util_hfsm&) = delete;

  /* nest acquisition metrics */
  RCPPSW_WRAP_DECL_OVERRIDE(const cssnest::acq::base_acq*,
                            nest_acq_strategy,
                            const);

  /* block drop metrics */
  RCPPSW_WRAP_DECL_OVERRIDE(const cssblocks::drop::base_drop*,
                            block_drop_strategy,
                            const);

 protected:
  struct nest_transport_data : public rpfsm::event_data {
    explicit nest_transport_data(const rmath::vector2d& in)
        : nest_loc(in) {}
    rmath::vector2d nest_loc;
  };

  strategy_set& strategies(void) { return m_strategies; }

  /**
   * \brief Update the \ref interference_tracker state and applies the avoidance
   * force if obstacles are detected.
   */
  void inta_state_update(void);

  /**
   * \brief Update the \ref nest_zone_tracker state.
   */
  void nz_state_update(void);

  /**
   * \brief Robots entering this state will return to the nest using the
   * specified acquisition strategy.
   *
   * This state MUST have a parent state defined that is not
   * \ref rcppsw::patterns::fsm::hfsm::top_state().
   */
  RCPPSW_HFSM_STATE_DECLARE(foraging_util_hfsm,
                            transport_to_nest,
                            nest_transport_data);

  /**
   * \brief Robots entering this state will drop their carried block in the nest
   * using the specified strategy.
   *
   * This state MUST have a parent state defined that is not \ref
   * hfsm::top_state().
   */
  RCPPSW_HFSM_STATE_DECLARE(foraging_util_hfsm, drop_carried_block, rpfsm::event_data);

  /**
   * \brief Robots entering this state will leave the nest (they are assumed to
   * already be in the nest when this state is entered).
   *
   * This state MUST have a parent state defined that is not \ref
   * hfsm::top_state().
   */
  RCPPSW_HFSM_STATE_DECLARE(foraging_util_hfsm, leaving_nest, rpfsm::event_data);

  /**
   * \brief Entry state for returning to nest.
   *
   * Used to:
   *
   * - Set LED colors for visualization purposes.
   * - Enable light sensor (disabled otherwise for computational
   *   efficiency).
   */
  RCPPSW_HFSM_ENTRY_DECLARE_ND(foraging_util_hfsm, entry_transport_to_nest);

  /**
   * \brief A simple entry state for leaving nest, used to set LED colors for
   * visualization purposes and enable light sensors.
   */
  RCPPSW_HFSM_ENTRY_DECLARE_ND(foraging_util_hfsm, entry_leaving_nest);

  /**
   * \brief A simple entry state for leaving nest, used to enable light sensors.
   */
  RCPPSW_HFSM_EXIT_DECLARE(foraging_util_hfsm, exit_leaving_nest);

  /**
   * \brief Exit state for returning to nest (i.e. when the robot arrives in the
   * nest).
   *
   * Used to:
   *
   * - Set LED colors for visualization purposes.
   * - Disable light sensor (disabled unless a robot is activiely returning to
   *   the nest for computational efficiency).
   */
  RCPPSW_HFSM_EXIT_DECLARE(foraging_util_hfsm, exit_transport_to_nest);

 private:
  /* clang-format off */
  strategy_set m_strategies;
  /* clang-format on */
};

NS_END(fsm, foraging, cosm);
