/**
 * \file diff_drive_fsm.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
 * Includesp
 ******************************************************************************/
#include "cosm/kin2D/diff_drive_fsm.hpp"

#include "rcppsw/math/range.hpp"
#include "rcppsw/math/angles.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, kin2D);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
diff_drive_fsm::diff_drive_fsm(double max_speed,
                               const rmath::radians& soft_turn_max)
    : rpfsm::simple_fsm(ekST_MAX_STATES),
      mc_max_speed(max_speed),
      mc_soft_turn_max(soft_turn_max),
      RCPPSW_FSM_DEFINE_STATE_MAP(mc_state_map,
                                  RCPPSW_FSM_STATE_MAP_ENTRY(&soft_turn),
                                  RCPPSW_FSM_STATE_MAP_ENTRY(&hard_turn)) {}

diff_drive_fsm::diff_drive_fsm(const diff_drive_fsm& other)
    : rpfsm::simple_fsm(ekST_MAX_STATES),
      mc_max_speed(other.mc_max_speed),
      mc_soft_turn_max(other.mc_soft_turn_max),
      RCPPSW_FSM_DEFINE_STATE_MAP(mc_state_map,
                                  RCPPSW_FSM_STATE_MAP_ENTRY(&soft_turn),
                                  RCPPSW_FSM_STATE_MAP_ENTRY(&hard_turn)) {}

/*******************************************************************************
 * Events
 ******************************************************************************/
void diff_drive_fsm::change_velocity(const rmath::vector2d& old_vel,
                                     const rmath::vector2d& new_vel) {
  RCPPSW_FSM_DEFINE_TRANSITION_MAP(kTRANSITIONS){
    ekST_SOFT_TURN, /* slow turn */
    ekST_HARD_TURN, /* hard turn */
  };
  RCPPSW_FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()],
                 std::make_unique<turn_data>(new_vel.length(),
                                             (old_vel - new_vel).angle()));
} /* set_rel_heading() */

/*******************************************************************************
 * States
 ******************************************************************************/
RCPPSW_FSM_STATE_DEFINE(diff_drive_fsm, soft_turn, turn_data* data) {
  rmath::range<rmath::radians> range(-mc_soft_turn_max, mc_soft_turn_max);

  rmath::radians angle = data->angle;
  /* too large of a direction change for soft turn--go to hard turn */
  if (!range.contains(angle.signed_normalize())) {
    internal_event(ekST_HARD_TURN);
    return rpfsm::event_signal::ekHANDLED;
  }

  double clamped = std::min(data->speed, mc_max_speed);
  configure_twist(clamped, data->angle);

  return rpfsm::event_signal::ekHANDLED;
}
RCPPSW_FSM_STATE_DEFINE(diff_drive_fsm, hard_turn, turn_data* data) {
  rmath::range<rmath::radians> range(-mc_soft_turn_max, mc_soft_turn_max);

  /* too little of a direction change for hard turn--go to soft turn */
  if (range.contains(data->angle.signed_normalize())) {
    internal_event(ekST_SOFT_TURN);
    return rpfsm::event_signal::ekHANDLED;
  }
  /* configure_wheel_speeds(-mc_max_speed, mc_max_speed, data->angle); */
  configure_twist(mc_max_speed, data->angle);

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

NS_END(kin2D, cosm);
