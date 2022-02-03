/**
 * \file explore_for_goal_fsm.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/fsm/explore_for_goal_fsm.hpp"

#include "cosm/spatial/fsm/util_signal.hpp"
#include "cosm/subsystem/actuation_subsystem2D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
explore_for_goal_fsm::explore_for_goal_fsm(
    const csfsm::fsm_params* params,
    std::unique_ptr<csstrategy::base_strategy> behavior,
    rmath::rng* rng,
    const std::function<bool(void)>& goal_detect)
    : util_hfsm(params, rng, ekST_MAX_STATES),
      ER_CLIENT_INIT("cosm.spatial.fsm.explore_for_goal"),
      RCPPSW_HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(explore, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      RCPPSW_HFSM_DEFINE_STATE_MAP(
          mc_state_map,
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&start),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&explore,
                                             nullptr,
                                             &entry_explore,
                                             nullptr),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&finished)),
      m_explore_behavior(std::move(behavior)),
      m_goal_detect(goal_detect) {}

RCPPSW_HFSM_STATE_DEFINE_ND(explore_for_goal_fsm, start) {
  if (ekST_START != last_state()) {
    ER_DEBUG("Executing ekST_START");
  }
  internal_event(ekST_EXPLORE);
  return util_signal::ekHANDLED;
}

RCPPSW_CONST RCPPSW_HFSM_STATE_DEFINE_ND(explore_for_goal_fsm, finished) {
  if (ekST_FINISHED != last_state()) {
    ER_DEBUG("Executing ekST_FINISHED");
  }
  return util_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE_ND(explore_for_goal_fsm, explore) {
  if (ekST_EXPLORE != last_state()) {
    ER_DEBUG("Executing ekST_EXPLORE");
  }

  if (m_explore_time >= kMIN_EXPLORE_TIME && m_goal_detect()) {
    internal_event(ekST_FINISHED);
  } else {
    if (nullptr != m_explore_behavior) {
      m_explore_behavior->task_execute();
    }
    ++m_explore_time;
  }
  return util_signal::ekHANDLED;
}

RCPPSW_HFSM_ENTRY_DEFINE_ND(explore_for_goal_fsm, entry_explore) {
  actuation()->diagnostics()->emit(chactuators::diagnostics::ekEXPLORE);
}

/*******************************************************************************
 * Taskable Interface
 ******************************************************************************/
bool explore_for_goal_fsm::task_finished(void) const {
  return ekST_FINISHED == current_state();
}

bool explore_for_goal_fsm::task_running(void) const {
  return ekST_EXPLORE == current_state() && nullptr != m_explore_behavior;
}

void explore_for_goal_fsm::task_execute(void) {
  inject_event(util_signal::ekRUN, rpfsm::event_type::ekNORMAL);
} /* task_execute() */

void explore_for_goal_fsm::task_reset(void) {
  init();
  m_explore_time = 0;
  if (nullptr != m_explore_behavior) {
    m_explore_behavior->task_reset();
  }
}

void explore_for_goal_fsm::task_start(ta::taskable_argument* c_arg) {
  if (nullptr != m_explore_behavior) {
    m_explore_behavior->task_start(c_arg);
  }

  static const uint8_t kTRANSITIONS[] = {
    ekST_EXPLORE, /* start */
    ekST_EXPLORE, /* explore */
    ekST_EXPLORE, /* finished */
  };
  RCPPSW_HFSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()]);
}

NS_END(fsm, spatial, cosm);
