/**
 * \file acquire_goal_fsm.cpp
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
#include "cosm/spatial/fsm/acquire_goal_fsm.hpp"

#include "cosm/spatial/fsm/point_argument.hpp"
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
acquire_goal_fsm::acquire_goal_fsm(
    const csfsm::fsm_params* params,
    std::unique_ptr<csstrategy::base_strategy> behavior,
    rmath::rng* rng,
    const struct hook_list& hooks)
    : util_hfsm(params, rng, ekST_MAX_STATES),
      ER_CLIENT_INIT("cosm.spatial.fsm.acquire_goal_fsm"),
      RCPPSW_HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(fsm_acquire_goal, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(finished, hfsm::top_state()),
      RCPPSW_HFSM_DEFINE_STATE_MAP(
          mc_state_map,
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&start),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&fsm_acquire_goal,
                                             nullptr,
                                             nullptr,
                                             &exit_fsm_acquire_goal),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX(&finished)),
      m_hooks(hooks),
      m_vector_fsm(params, rng),
      m_explore_fsm(params, std::move(behavior), rng, m_hooks.explore_term_cb) {
  m_explore_fsm.change_parent(explore_for_goal_fsm::state::ekST_EXPLORE,
                              &fsm_acquire_goal);
}

RCPPSW_HFSM_STATE_DEFINE_ND(acquire_goal_fsm, start) {
  ER_DEBUG("Executing ekST_START");
  internal_event(ekST_ACQUIRE_GOAL);
  return util_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE_ND(acquire_goal_fsm, fsm_acquire_goal) {
  if (ekST_ACQUIRE_GOAL != last_state()) {
    ER_DEBUG("Executing ekST_ACQUIRE_GOAL");
  }

  if (acquire_goal()) {
    internal_event(ekST_FINISHED);
  }
  return rpfsm::event_signal::ekHANDLED;
}

RCPPSW_HFSM_EXIT_DEFINE(acquire_goal_fsm, exit_fsm_acquire_goal) {
  m_vector_fsm.task_reset();
  m_explore_fsm.task_reset();
}

RCPPSW_CONST RCPPSW_HFSM_STATE_DEFINE_ND(acquire_goal_fsm, finished) {
  if (ekST_FINISHED != last_state()) {
    ER_DEBUG("Executing ekST_FINISHED");
  }

  return rpfsm::event_signal::ekHANDLED;
}

/*******************************************************************************
 * FSM Metrics
 ******************************************************************************/
bool acquire_goal_fsm::exp_interference(void) const {
  return (m_explore_fsm.task_running() && m_explore_fsm.exp_interference()) ||
         (m_vector_fsm.task_running() && m_vector_fsm.exp_interference());
} /* exp_interference() */

bool acquire_goal_fsm::entered_interference(void) const {
  return (m_explore_fsm.task_running() && m_explore_fsm.entered_interference()) ||
         (m_vector_fsm.task_running() && m_vector_fsm.entered_interference());
} /* entered_interference() */

bool acquire_goal_fsm::exited_interference(void) const {
  return (m_explore_fsm.task_running() && m_explore_fsm.exited_interference()) ||
         (m_vector_fsm.task_running() && m_vector_fsm.exited_interference());
} /* exited_interference() */

rtypes::timestep acquire_goal_fsm::interference_duration(void) const {
  if (m_explore_fsm.task_running()) {
    return m_explore_fsm.interference_duration();
  } else if (m_vector_fsm.task_running()) {
    return m_vector_fsm.interference_duration();
  }
  return rtypes::timestep(0);
} /* interference_duration() */

bool acquire_goal_fsm::goal_acquired(void) const {
  return current_state() == ekST_FINISHED;
} /* cache_acquired() */

acquire_goal_fsm::exp_status acquire_goal_fsm::is_exploring_for_goal(void) const {
  return exp_status{ current_state() == ekST_ACQUIRE_GOAL &&
                         m_explore_fsm.task_running(),
                     !m_hooks.candidates_exist() };
} /* is_exploring_for_goal() */

bool acquire_goal_fsm::is_vectoring_to_goal(void) const {
  return current_state() == ekST_ACQUIRE_GOAL && m_vector_fsm.task_running();
} /* is_vectoring_to_goal() */

metrics::goal_acq_metrics::goal_type
acquire_goal_fsm::acquisition_goal(void) const {
  if (ekST_ACQUIRE_GOAL == current_state()) {
    return m_hooks.acquisition_goal();
  }
  return metrics::goal_acq_metrics::goal_type(-1);
} /* acquisition_goal() */

rmath::vector3z acquire_goal_fsm::acquisition_loc3D(void) const {
  return sensing()->dpos3D();
} /* acquisition_loc3D() */

rmath::vector3z acquire_goal_fsm::explore_loc3D(void) const {
  return sensing()->dpos3D();
} /* explore_loc3D() */

rmath::vector3z acquire_goal_fsm::vector_loc3D(void) const {
  return sensing()->dpos3D();
} /* vector_loc3D() */

rmath::vector3z acquire_goal_fsm::interference_loc3D(void) const {
  return sensing()->dpos3D();
} /* interference_loc3D() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void acquire_goal_fsm::init(void) {
  util_hfsm::init();
  m_vector_fsm.task_reset();
  m_explore_fsm.task_reset();
  m_acq_id = rtypes::constants::kNoUUID;
} /* init() */

bool acquire_goal_fsm::acquire_goal(void) {
  /*
   * If we know of goal caches in the arena, go to the location of the best
   * one. Otherwise, explore until you find one. If during exploration we find
   * one through our LOS, then stop exploring and go vector to it.
   */
  if (!acquire_known_goal()) {
    if (m_vector_fsm.task_running()) {
      return false;
    }

    /*
     * When we got to our chosen goal it was found to be unsuitable and we
     * currently don't know of any other candidates, so we have to explore to
     * find what we want.
     */
    return acquire_unknown_goal();
  }
  return true;
} /* acquire_goal() */

bool acquire_goal_fsm::acquire_unknown_goal(void) {
  if (!m_explore_fsm.task_running()) {
    m_explore_fsm.task_reset();
    m_explore_fsm.task_start(nullptr);
  }
  m_explore_fsm.task_execute();
  if (m_explore_fsm.task_finished()) {
    return m_hooks.goal_acquired_cb(true);
  }
  return false;
} /* acquire_unknown_goal() */

bool acquire_goal_fsm::acquire_known_goal(void) {
  /*
   * If we don't know of any blocks and we are not current vectoring towards
   * one, then there is no way we can acquire a known block, so bail out.
   */
  if (!m_hooks.candidates_exist() && !m_vector_fsm.task_running()) {
    return false;
  }

  /*
   * We know of candidates but are not vectoring towards any of them. So pick
   * one and start the vector FSM.
   */
  if (m_hooks.candidates_exist() && !m_vector_fsm.task_running()) {
    /*
     * If we get here, we must know of some candidates/perceived entities of
     * interest, but not be currently vectoring toward any of them.
     */
    auto selection = m_hooks.goal_select();

    /*
     * If this happens, all the entities we know of are too close for us to
     * vector to, or there was some other issue with selecting one.
     */
    if (!selection) {
      return false;
    }

    m_explore_fsm.task_reset();
    m_vector_fsm.task_reset();

    ER_INFO("Start acquiring goal@%s tol=%f",
            std::get<0>(selection.get()).to_str().c_str(),
            std::get<1>(selection.get()));
    point_argument v(std::get<1>(selection.get()), std::get<0>(selection.get()));
    m_acq_id = std::get<2>(selection.get());
    m_vector_fsm.task_start(&v);
    m_first_acq_step = true;
    return false;
  }

  /* First timestep of acquisition */
  if (m_first_acq_step && m_hooks.begin_acq_cb) {
    m_hooks.begin_acq_cb();
  }

  /* we are vectoring */
  if (!m_vector_fsm.task_finished()) {
    m_vector_fsm.task_execute();
    if (!m_hooks.goal_valid_cb(m_vector_fsm.target(), m_acq_id)) {
      m_vector_fsm.task_reset();
      return false;
    }
  }

  if (m_vector_fsm.task_finished()) {
    m_vector_fsm.task_reset();
    return m_hooks.goal_acquired_cb(false);
  }
  return false;
} /* acquire_known_goal() */

void acquire_goal_fsm::task_execute(void) {
  inject_event(util_signal::ekRUN, rpfsm::event_type::ekNORMAL);
} /* task_execute() */

NS_END(fsm, spatial, cosm);
