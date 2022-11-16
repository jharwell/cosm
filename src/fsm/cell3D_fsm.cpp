/**
 * \file cell3D_fsm.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/fsm/cell3D_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, fsm);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cell3D_fsm::cell3D_fsm(void)
    : rpfsm::simple_fsm(state::ekST_MAX_STATES, state::ekST_UNKNOWN),
      ER_CLIENT_INIT("cosm.fsm.cell3D"),
      RCPPSW_FSM_DEFINE_STATE_MAP(
          mc_state_map,
          RCPPSW_FSM_STATE_MAP_ENTRY(&state_unknown),
          RCPPSW_FSM_STATE_MAP_ENTRY(&state_empty),
          RCPPSW_FSM_STATE_MAP_ENTRY(&state_block),
          RCPPSW_FSM_STATE_MAP_ENTRY(&state_block_extent)) {}

cell3D_fsm::cell3D_fsm(const cell3D_fsm&)
    : rpfsm::simple_fsm(state::ekST_MAX_STATES, state::ekST_UNKNOWN),
      ER_CLIENT_INIT("cosm.fsm.cell3D"),
      RCPPSW_FSM_DEFINE_STATE_MAP(
          mc_state_map,
          RCPPSW_FSM_STATE_MAP_ENTRY(&state_unknown),
          RCPPSW_FSM_STATE_MAP_ENTRY(&state_empty),
          RCPPSW_FSM_STATE_MAP_ENTRY(&state_block),
          RCPPSW_FSM_STATE_MAP_ENTRY(&state_block_extent)) {}

/*******************************************************************************
 * Event Functions
 ******************************************************************************/
void cell3D_fsm::event_unknown(void) {
  RCPPSW_FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    state::ekST_UNKNOWN, /* unknown */
    state::ekST_UNKNOWN, /* empty */
    state::ekST_UNKNOWN, /* has block */
    state::ekST_UNKNOWN, /* block extent */
  };
  RCPPSW_FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, state::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_unknown() */

void cell3D_fsm::event_empty(void) {
  RCPPSW_FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    state::ekST_EMPTY, /* unknown */
    state::ekST_EMPTY, /* empty */
    state::ekST_EMPTY, /* has block */
    state::ekST_EMPTY, /* block extent */
  };
  RCPPSW_FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, state::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_empty() */

void cell3D_fsm::event_block_place(void) {
  RCPPSW_FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    state::ekST_HAS_BLOCK, /* unknown */
    state::ekST_HAS_BLOCK, /* empty */
    rpfsm::event_signal::ekFATAL, /* has block */
    rpfsm::event_signal::ekFATAL, /* block extent */
  };
  RCPPSW_FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, state::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_block_place() */

void cell3D_fsm::event_block_extent(void) {
  RCPPSW_FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    state::ekST_BLOCK_EXTENT, /* unknown */
    state::ekST_BLOCK_EXTENT, /* empty */
    rpfsm::event_signal::ekFATAL, /* has block */
    rpfsm::event_signal::ekFATAL, /* block extent */
  };
  RCPPSW_FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, state::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_block_extent() */

/*******************************************************************************
 * State Functions
 ******************************************************************************/
RCPPSW_FSM_STATE_DEFINE_ND(cell3D_fsm, state_unknown) {
  return rpfsm::event_signal::ekHANDLED;
}
RCPPSW_FSM_STATE_DEFINE_ND(cell3D_fsm, state_empty) {
  return rpfsm::event_signal::ekHANDLED;
}

RCPPSW_FSM_STATE_DEFINE_ND(cell3D_fsm, state_block) {
  return rpfsm::event_signal::ekHANDLED;
}

RCPPSW_FSM_STATE_DEFINE_ND(cell3D_fsm, state_block_extent) {
  return rpfsm::event_signal::ekHANDLED;
}

NS_END(fsm, cosm);
