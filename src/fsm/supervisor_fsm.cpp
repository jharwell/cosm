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

#include "cosm/fsm/util_signal.hpp"
#include "cosm/kin2D/governed_diff_drive.hpp"
#include "cosm/subsystem/actuation_subsystem2D.hpp"
#include "cosm/subsystem/saa_subsystem2D.hpp"
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
struct normal_visitor {
  void operator()(ta::base_executive* executive) const { executive->run(); }
  void operator()(ta::taskable* taskable) const { taskable->task_execute(); }
};

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
supervisor_fsm::supervisor_fsm(csubsystem::saa_subsystem2D* const saa)
    : rpfsm::simple_fsm(states::ekST_MAX_STATES, states::ekST_START),
      ER_CLIENT_INIT("cosm.fsm.supervisor"),
      FSM_DEFINE_STATE_MAP(mc_state_map,
                           FSM_STATE_MAP_ENTRY(&start),
                           FSM_STATE_MAP_ENTRY(&normal),
                           FSM_STATE_MAP_ENTRY(&malfunction)),
      m_saa(saa) {}

/*******************************************************************************
 * States
 ******************************************************************************/
RCSW_CONST FSM_STATE_DEFINE_ND(supervisor_fsm, start) {
  internal_event(ekST_NORMAL);
  return cfsm::util_signal::ekHANDLED;
}

RCSW_CONST FSM_STATE_DEFINE_ND(supervisor_fsm, normal) {
  boost::apply_visitor(normal_visitor(), m_variant);
  m_saa->steer_force2D_apply();
  return fsm::util_signal::ekHANDLED;
}

RCSW_CONST FSM_STATE_DEFINE_ND(supervisor_fsm, malfunction) {
  m_saa->actuation()->actuator<ckin2D::governed_diff_drive>()->reset();
  return fsm::util_signal::ekHANDLED;
}

/*******************************************************************************
 * Event Functions
 ******************************************************************************/
void supervisor_fsm::event_malfunction(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    /* Possible to have a malfunction event on first timestep  */
      states::ekST_MALFUNCTION,  /* start */
      states::ekST_MALFUNCTION,  /* normal */
      fsm::util_signal::ekFATAL, /* malfunction */
  };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, states::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_malfunction() */

void supervisor_fsm::event_repair(void) {
  FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    /* Possible to have repair malfunction event on first timestep  */
      states::ekST_NORMAL,       /* start */
      fsm::util_signal::ekFATAL, /* normal */
      states::ekST_NORMAL,       /* malfunction */
  };
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, states::ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()], nullptr);
} /* event_repair() */

NS_END(fsm, cosm);
