/**
 * \file supervisor_fsm.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/fsm/supervisor_fsm.hpp"

#include "rcppsw/patterns/fsm/event.hpp"

#include "cosm/hal/subsystem/base_actuation_subsystem.hpp"
#include "cosm/subsystem/base_saa_subsystem.hpp"
#include "cosm/subsystem/sensing_subsystem.hpp"
#include "cosm/ta/base_executive.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::fsm {

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
supervisor_fsm::supervisor_fsm(csubsystem::base_saa_subsystem* saa)
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
  m_saa->apf_apply();
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

} /* namespace cosm::fsm */
