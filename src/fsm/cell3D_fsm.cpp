/**
 * \file cell3D_fsm.cpp
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
#include "cosm/fsm/cell3D_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, fsm);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cell3D_fsm::cell3D_fsm(void)
    : rpfsm::simple_fsm(states::ekST_MAX_STATES, states::ekST_UNKNOWN),
      ER_CLIENT_INIT("cosm.fsm.cell3D"),
      FSM_DEFINE_STATE_MAP(mc_state_map,
                           FSM_STATE_MAP_ENTRY(&state_unknown),
                           FSM_STATE_MAP_ENTRY(&state_empty),
                           FSM_STATE_MAP_ENTRY(&state_block),
                           FSM_STATE_MAP_ENTRY(&state_block_extent)) {}

cell3D_fsm::cell3D_fsm(const cell3D_fsm&)
    : rpfsm::simple_fsm(states::ekST_MAX_STATES, states::ekST_UNKNOWN),
    ER_CLIENT_INIT("cosm.fsm.cell3D"),
    FSM_DEFINE_STATE_MAP(mc_state_map,
                         FSM_STATE_MAP_ENTRY(&state_unknown),
                         FSM_STATE_MAP_ENTRY(&state_empty),
                         FSM_STATE_MAP_ENTRY(&state_block),
                         FSM_STATE_MAP_ENTRY(&state_block_extent)) {}

/*******************************************************************************
 * Event Functions
 ******************************************************************************/
void cell3D_fsm::event_unknown(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
      states::ekST_UNKNOWN, /* unknown */
      states::ekST_UNKNOWN, /* empty */
      states::ekST_UNKNOWN, /* has block */
      states::ekST_UNKNOWN, /* block extent */
  };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, states::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_unknown() */

void cell3D_fsm::event_empty(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
      states::ekST_EMPTY, /* unknown */
      states::ekST_EMPTY, /* empty */
      states::ekST_EMPTY, /* has block */
      states::ekST_EMPTY, /* block extent */
  };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, states::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_empty() */

void cell3D_fsm::event_block_place(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
      states::ekST_HAS_BLOCK,       /* unknown */
      states::ekST_HAS_BLOCK,       /* empty */
      rpfsm::event_signal::ekFATAL, /* has block */
      rpfsm::event_signal::ekFATAL, /* block extent */
  };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, states::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_block_place() */

void cell3D_fsm::event_block_extent(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
      states::ekST_BLOCK_EXTENT,    /* unknown */
      states::ekST_BLOCK_EXTENT,    /* empty */
      rpfsm::event_signal::ekFATAL, /* has block */
      rpfsm::event_signal::ekFATAL, /* block extent */
  };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, states::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_block_extent() */

/*******************************************************************************
 * State Functions
 ******************************************************************************/
FSM_STATE_DEFINE_ND(cell3D_fsm, state_unknown) {
  return rpfsm::event_signal::ekHANDLED;
}
FSM_STATE_DEFINE_ND(cell3D_fsm, state_empty) {
  return rpfsm::event_signal::ekHANDLED;
}

FSM_STATE_DEFINE_ND(cell3D_fsm, state_block) {
  return rpfsm::event_signal::ekHANDLED;
}

FSM_STATE_DEFINE_ND(cell3D_fsm, state_block_extent) {
  return rpfsm::event_signal::ekHANDLED;
}

NS_END(fsm, cosm);
