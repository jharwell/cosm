/**
 * \file diff_drive_fsm.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includesp
 ******************************************************************************/
#include "cosm/kin2D/diff_drive_fsm.hpp"

#include "rcppsw/math/angles.hpp"
#include "rcppsw/math/range.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::kin2D {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
diff_drive_fsm::diff_drive_fsm(const config::diff_drive_config* const config)
    : rpfsm::simple_fsm(ekST_MAX_STATES),
      mc_config(*config),
      RCPPSW_FSM_DEFINE_STATE_MAP(mc_state_map,
                                  RCPPSW_FSM_STATE_MAP_ENTRY(&soft_turn),
                                  RCPPSW_FSM_STATE_MAP_ENTRY(&hard_turn)) {}

diff_drive_fsm::diff_drive_fsm(diff_drive_fsm&& other)
    : rpfsm::simple_fsm(ekST_MAX_STATES),
      mc_config(other.mc_config),
      RCPPSW_FSM_DEFINE_STATE_MAP(mc_state_map,
                                  RCPPSW_FSM_STATE_MAP_ENTRY(&soft_turn),
                                  RCPPSW_FSM_STATE_MAP_ENTRY(&hard_turn)) {}

/*******************************************************************************
 * Events
 ******************************************************************************/
void diff_drive_fsm::change_velocity(const ckin::twist& delta) {
  RCPPSW_FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    ekST_SOFT_TURN, /* slow turn */
    ekST_HARD_TURN, /* hard turn */
  };
  RCPPSW_FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()],
                 std::make_unique<turn_data>(delta.linear.x(),
                                             rmath::radians(delta.angular.z())));
} /* change_velocity() */

/*******************************************************************************
 * States
 ******************************************************************************/
RCPPSW_FSM_STATE_DEFINE(diff_drive_fsm, soft_turn, turn_data* data) {
  rmath::range<rmath::radians> range(-mc_config.soft_turn_max,
                                     mc_config.soft_turn_max);

  rmath::radians angle = data->angle;
  /* too large of a direction change for soft turn--go to hard turn */
  if (!range.contains(angle.signed_normalize())) {
    internal_event(ekST_HARD_TURN);
    return rpfsm::event_signal::ekHANDLED;
  }
  double clamped_lin = std::min(data->speed, mc_config.max_linear_speed);
  double clamped_ang = std::min(data->angle.v(), mc_config.max_angular_speed);
  configure_twist(clamped_lin, rmath::radians(clamped_ang));

  return rpfsm::event_signal::ekHANDLED;
}
RCPPSW_FSM_STATE_DEFINE(diff_drive_fsm, hard_turn, turn_data* data) {
  rmath::range<rmath::radians> range(-mc_config.soft_turn_max,
                                     mc_config.soft_turn_max);

  rmath::radians angle = data->angle;

  /* too little of a direction change for hard turn--go to soft turn */
  if (range.contains(angle.signed_normalize())) {
    internal_event(ekST_SOFT_TURN);
    return rpfsm::event_signal::ekHANDLED;
  }
  /* hard turn=spin in place. But not too fast! */
  double clamped_ang = std::min(data->angle.v(), mc_config.max_angular_speed);
  configure_twist(0, rmath::radians(clamped_ang + 0.0001));

  return rpfsm::event_signal::ekHANDLED;
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void diff_drive_fsm::configure_twist(double speed,
                                     const rmath::radians& heading) {
  m_twist.linear = rmath::vector3d::X * speed;
  m_twist.angular = rmath::vector3d::Z * heading.v();
} /* configure_twist() */

} /* namespace cosm::kin2D */
