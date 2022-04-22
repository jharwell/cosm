/**
 * \file supervisor_fsm.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
#include "cosm/fsm/supervisor_fsm.hpp"

#include "rcppsw/patterns/fsm/event.hpp"

#include "cosm/kin2D/governed_diff_drive.hpp"
#include "cosm/subsystem/actuation_subsystem2D.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"
#include "cosm/ta/base_executive.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, fsm);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * \brief Disambiguate the type of the thing being supervised so that the
 * appropriate "do normal operation" function can be called.
 */
struct normal_op_visitor {
  void operator()(ta::base_executive* executive) const { executive->run(); }
  void operator()(ta::taskable* taskable) const { taskable->task_execute(); }
};

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
supervisor_fsm::supervisor_fsm(subsystem::saa_subsystemQ3D* saa)
    : rpfsm::simple_fsm(states::ekST_MAX_STATES, states::ekST_START),
      ER_CLIENT_INIT("cosm.fsm.supervisor"),
      RCPPSW_FSM_DEFINE_STATE_MAP(
          mc_state_map,
          RCPPSW_FSM_STATE_MAP_ENTRY_EX(&start),
          RCPPSW_FSM_STATE_MAP_ENTRY_EX(&normal),
          RCPPSW_FSM_STATE_MAP_ENTRY_EX(&malfunction),
          RCPPSW_FSM_STATE_MAP_ENTRY_EX_ALL(&stop, nullptr, &entry_stop, nullptr)),
      m_saa(saa) {}

/*******************************************************************************
 * States
 ******************************************************************************/
RCPPSW_CONST RCPPSW_FSM_STATE_DEFINE_ND(supervisor_fsm, start) {
  internal_event(ekST_NORMAL);
  return rpfsm::event_signal::ekHANDLED;
}

RCPPSW_CONST RCPPSW_FSM_STATE_DEFINE_ND(supervisor_fsm, normal) {
  std::visit(normal_op_visitor(), m_supervisee);
  m_saa->steer_force2D_apply();
  return rpfsm::event_signal::ekHANDLED;
}

RCPPSW_CONST RCPPSW_FSM_STATE_DEFINE_ND(supervisor_fsm, malfunction) {
  m_saa->actuation()->reset();
  return rpfsm::event_signal::ekHANDLED;
}

RCPPSW_CONST RCPPSW_FSM_STATE_DEFINE_ND(supervisor_fsm, stop) {
  return rpfsm::event_signal::ekHANDLED;
}

RCPPSW_FSM_ENTRY_DEFINE_ND(supervisor_fsm, entry_stop) {
  m_saa->actuation()->disable();
  m_saa->sensing()->disable();
}

/*******************************************************************************
 * Event Functions
 ******************************************************************************/
void supervisor_fsm::event_malfunction(void) {
  RCPPSW_FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    /* Possible to have a malfunction event on first timestep  */
    states::ekST_MALFUNCTION, /* start */
    states::ekST_MALFUNCTION, /* normal */
    rpfsm::event_signal::ekFATAL, /* malfunction */
    rpfsm::event_signal::ekFATAL, /* stop */
  };
  RCPPSW_FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, states::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_malfunction() */

void supervisor_fsm::event_repair(void) {
  RCPPSW_FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    /* Possible to have repair event on first timestep  */
    states::ekST_NORMAL, /* start */
    rpfsm::event_signal::ekFATAL, /* normal */
    states::ekST_NORMAL, /* malfunction */
    rpfsm::event_signal::ekFATAL, /* stop */
  };
  RCPPSW_FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, states::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_repair() */

void supervisor_fsm::event_stop(void) {
  RCPPSW_FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    /* Possible to have stop event on first timestep  */
    states::ekST_NORMAL, /* start */
    states::ekST_STOP, /* normal */
    states::ekST_STOP, /* malfunction */
    states::ekST_STOP, /* stop */
  };
  RCPPSW_FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, states::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_stop() */

NS_END(fsm, cosm);
