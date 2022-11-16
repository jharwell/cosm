/**
 * \file cell2D_fsm.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */
/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/fsm/cell2D_fsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, fsm);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cell2D_fsm::cell2D_fsm(void)
    : rpfsm::simple_fsm(state::ekST_MAX_STATES, state::ekST_UNKNOWN),
      ER_CLIENT_INIT("cosm.fsm.cell2D"),
      RCPPSW_FSM_DEFINE_STATE_MAP(
          mc_state_map,
          RCPPSW_FSM_STATE_MAP_ENTRY(&state_unknown),
          RCPPSW_FSM_STATE_MAP_ENTRY(&state_empty),
          RCPPSW_FSM_STATE_MAP_ENTRY(&state_block),
          RCPPSW_FSM_STATE_MAP_ENTRY(&state_cache),
          RCPPSW_FSM_STATE_MAP_ENTRY(&state_block_extent),
          RCPPSW_FSM_STATE_MAP_ENTRY(&state_cache_extent),
          RCPPSW_FSM_STATE_MAP_ENTRY(&state_nest_extent)) {}

cell2D_fsm::cell2D_fsm(const cell2D_fsm& other)
    : rpfsm::simple_fsm(state::ekST_MAX_STATES, state::ekST_UNKNOWN),
      ER_CLIENT_INIT("cosm.fsm.cell2D"),
      RCPPSW_FSM_DEFINE_STATE_MAP(mc_state_map,
                                  RCPPSW_FSM_STATE_MAP_ENTRY(&state_unknown),
                                  RCPPSW_FSM_STATE_MAP_ENTRY(&state_empty),
                                  RCPPSW_FSM_STATE_MAP_ENTRY(&state_block),
                                  RCPPSW_FSM_STATE_MAP_ENTRY(&state_cache),
                                  RCPPSW_FSM_STATE_MAP_ENTRY(&state_block_extent),
                                  RCPPSW_FSM_STATE_MAP_ENTRY(&state_cache_extent),
                                  RCPPSW_FSM_STATE_MAP_ENTRY(&state_nest_extent)),
      m_block_count(other.m_block_count) {}

/*******************************************************************************
 * Event Functions
 ******************************************************************************/
void cell2D_fsm::event_unknown(void) {
  RCPPSW_FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    state::ekST_UNKNOWN, /* unknown */
    state::ekST_UNKNOWN, /* empty */
    state::ekST_UNKNOWN, /* has block */
    state::ekST_UNKNOWN, /* has cache */
    state::ekST_UNKNOWN, /* block extent */
    state::ekST_UNKNOWN, /* cache extent */
    state::ekST_UNKNOWN, /* nest extent */
  };
  RCPPSW_FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, state::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_unknown() */

void cell2D_fsm::event_empty(void) {
  RCPPSW_FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    state::ekST_EMPTY, /* unknown */
    state::ekST_EMPTY, /* empty */
    state::ekST_EMPTY, /* has block */
    state::ekST_EMPTY, /* has cache */
    state::ekST_EMPTY, /* block extent */
    state::ekST_EMPTY, /* cache extent */
    rpfsm::event_signal::ekFATAL, /* nest extent */
  };
  RCPPSW_FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, state::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_empty() */

void cell2D_fsm::event_block_drop(void) {
  RCPPSW_FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    state::ekST_HAS_BLOCK, /* unknown */
    state::ekST_HAS_BLOCK, /* empty */
    state::ekST_HAS_CACHE, /* has block */
    state::ekST_HAS_CACHE, /* has cache */
    rpfsm::event_signal::ekFATAL, /* block extent */
    rpfsm::event_signal::ekFATAL, /* cache extent */
    rpfsm::event_signal::ekFATAL, /* nest extent */
  };
  RCPPSW_FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, state::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()],
                 std::make_unique<block_data>(false));
} /* event_empty() */

void cell2D_fsm::event_block_pickup(void) {
  RCPPSW_FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    rpfsm::event_signal::ekFATAL, /* unknown */
    rpfsm::event_signal::ekFATAL, /* empty */
    state::ekST_EMPTY, /* has block */
    state::ekST_HAS_CACHE, /* has cache */
    state::ekST_EMPTY, /* block extent */
    rpfsm::event_signal::ekFATAL, /* cache extent */
    rpfsm::event_signal::ekFATAL, /* nest extent */
  };
  RCPPSW_FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, state::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()],
                 std::make_unique<block_data>(true));
} /* event_block_pickup() */

void cell2D_fsm::event_block_extent(void) {
  RCPPSW_FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    state::ekST_BLOCK_EXTENT, /* unknown */
    state::ekST_BLOCK_EXTENT, /* empty */
    rpfsm::event_signal::ekFATAL, /* has block */
    rpfsm::event_signal::ekFATAL, /* has cache */
    rpfsm::event_signal::ekFATAL, /* block extent */
    rpfsm::event_signal::ekFATAL, /* cache extent */
    rpfsm::event_signal::ekFATAL, /* nest extent */
  };
  RCPPSW_FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, state::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_block_extent() */

void cell2D_fsm::event_cache_extent(void) {
  RCPPSW_FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    state::ekST_CACHE_EXTENT, /* unknown */
    state::ekST_CACHE_EXTENT, /* empty */
    /*
       * This is technically bad, but the arena map fixes it right after
       * creating a new cache, so we can let it slide here.
       */
    state::ekST_CACHE_EXTENT, /* has block */
    rpfsm::event_signal::ekFATAL, /* has cache */
    rpfsm::event_signal::ekFATAL, /* block extent */
    rpfsm::event_signal::ekFATAL, /* cache extent */
    rpfsm::event_signal::ekFATAL, /* nest extent */
  };
  RCPPSW_FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, state::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_cache_extent() */

void cell2D_fsm::event_nest_extent(void) {
  RCPPSW_FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    state::ekST_NEST_EXTENT, /* unknown */
    state::ekST_NEST_EXTENT, /* empty */
    rpfsm::event_signal::ekFATAL, /* has block */
    rpfsm::event_signal::ekFATAL, /* has cache */
    rpfsm::event_signal::ekFATAL, /* block extent */
    rpfsm::event_signal::ekFATAL, /* cache extent */
    rpfsm::event_signal::ekFATAL, /* nest extent */
  };
  RCPPSW_FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, state::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_nest_extent() */

/*******************************************************************************
 * State Functions
 ******************************************************************************/
RCPPSW_FSM_STATE_DEFINE_ND(cell2D_fsm, state_unknown) {
  if (state::ekST_UNKNOWN != last_state()) {
    m_block_count = 0;
  }
  return rpfsm::event_signal::ekHANDLED;
}
RCPPSW_FSM_STATE_DEFINE_ND(cell2D_fsm, state_empty) {
  if (state::ekST_EMPTY != last_state()) {
    m_block_count = 0;
  }
  return rpfsm::event_signal::ekHANDLED;
}

RCPPSW_FSM_STATE_DEFINE_ND(cell2D_fsm, state_block) {
  if (state::ekST_HAS_BLOCK != last_state()) {
    m_block_count = 1;
  }
  return rpfsm::event_signal::ekHANDLED;
}

RCPPSW_FSM_STATE_DEFINE(cell2D_fsm, state_cache, struct block_data* data) {
  if (state::ekST_HAS_CACHE != last_state()) {
    ER_ASSERT(1 == m_block_count,
              "Incorrect block count: %zu vs %u",
              m_block_count,
              1U);
  }
  if (nullptr != data) {
    if (data->pickup) {
      --m_block_count;
    } else {
      ++m_block_count;
    }
  }
  if (1 == m_block_count) {
    internal_event(state::ekST_HAS_BLOCK);
  }
  return rpfsm::event_signal::ekHANDLED;
}

RCPPSW_FSM_STATE_DEFINE_ND(cell2D_fsm, state_cache_extent) {
  if (state::ekST_CACHE_EXTENT != last_state()) {
    m_block_count = 0;
  }
  return rpfsm::event_signal::ekHANDLED;
}

RCPPSW_FSM_STATE_DEFINE_ND(cell2D_fsm, state_block_extent) {
  if (state::ekST_BLOCK_EXTENT != last_state()) {
    m_block_count = 1;
  }
  return rpfsm::event_signal::ekHANDLED;
}

RCPPSW_FSM_STATE_DEFINE_ND(cell2D_fsm, state_nest_extent) {
  return rpfsm::event_signal::ekHANDLED;
}

NS_END(fsm, cosm);
