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
#include "cosm/fsm/explore_for_goal_fsm.hpp"

#include "cosm/fsm/util_signal.hpp"
#include "cosm/subsystem/actuation_subsystem2D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
explore_for_goal_fsm::explore_for_goal_fsm(
    subsystem::saa_subsystemQ3D* const saa,
    std::unique_ptr<expstrat::base_expstrat> behavior,
    rmath::rng* rng,
    const std::function<bool(void)>& goal_detect)
    : util_hfsm(saa, rng, ekST_MAX_STATES),
      ER_CLIENT_INIT("cosm.fsm.explore_for_goal"),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(explore, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      HFSM_DEFINE_STATE_MAP(
          mc_state_map,
          HFSM_STATE_MAP_ENTRY_EX(&start),
          HFSM_STATE_MAP_ENTRY_EX_ALL(&explore, nullptr, &entry_explore, nullptr),
          HFSM_STATE_MAP_ENTRY_EX(&finished)),
      m_explore_behavior(std::move(behavior)),
      m_goal_detect(goal_detect) {}

HFSM_STATE_DEFINE_ND(explore_for_goal_fsm, start) {
  internal_event(ekST_EXPLORE);
  return util_signal::ekHANDLED;
}

RCSW_CONST HFSM_STATE_DEFINE_ND(explore_for_goal_fsm, finished) {
  return util_signal::ekHANDLED;
}

HFSM_STATE_DEFINE_ND(explore_for_goal_fsm, explore) {
  if (ekST_EXPLORE != last_state()) {
    ER_DEBUG("Executing ekST_EXPLORE");
    m_explore_time = 0;
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

HFSM_ENTRY_DEFINE_ND(explore_for_goal_fsm, entry_explore) {
  actuation()->actuator<hal::actuators::led_actuator>()->set_color(
      -1, rutils::color::kMAGENTA);
}

/*******************************************************************************
 * Collision Metrics
 ******************************************************************************/
RCPPSW_WRAP_OVERRIDE_DEF(explore_for_goal_fsm,
                         in_collision_avoidance,
                         *m_explore_behavior,
                         const);
RCPPSW_WRAP_OVERRIDE_DEF(explore_for_goal_fsm,
                         entered_collision_avoidance,
                         *m_explore_behavior,
                         const);
RCPPSW_WRAP_OVERRIDE_DEF(explore_for_goal_fsm,
                         exited_collision_avoidance,
                         *m_explore_behavior,
                         const);
RCPPSW_WRAP_OVERRIDE_DEF(explore_for_goal_fsm,
                         collision_avoidance_duration,
                         *m_explore_behavior,
                         const);

RCPPSW_WRAP_OVERRIDE_DEF(explore_for_goal_fsm,
                         avoidance_loc2D,
                         *m_explore_behavior,
                         const);

RCPPSW_WRAP_OVERRIDE_DEF(explore_for_goal_fsm,
                         avoidance_loc3D,
                         *m_explore_behavior,
                         const);

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
bool explore_for_goal_fsm::task_running(void) const {
  return ekST_START != current_state() && ekST_FINISHED != current_state() &&
         nullptr != m_explore_behavior;
}

void explore_for_goal_fsm::task_execute(void) {
  inject_event(util_signal::ekRUN, rpfsm::event_type::ekNORMAL);
} /* task_execute() */

NS_END(fsm, cosm);
