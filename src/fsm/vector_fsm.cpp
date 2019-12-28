/**
 * \file vector_fsm.cpp
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
 * Includes
 ******************************************************************************/
#include "cosm/fsm/vector_fsm.hpp"

#include "cosm/fsm/point_argument.hpp"
#include "cosm/fsm/util_signal.hpp"
#include "cosm/subsystem/actuation_subsystem2D.hpp"
#include "cosm/subsystem/saa_subsystem2D.hpp"
#include "cosm/subsystem/sensing_subsystem2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
vector_fsm::vector_fsm(subsystem::saa_subsystem2D* const saa, rmath::rng* rng)
    : util_hfsm(saa, rng, ekST_MAX_STATES),
      ER_CLIENT_INIT("cosm.fsm.vector"),
      HFSM_CONSTRUCT_STATE(new_direction, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(vector, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(collision_avoidance, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(collision_recovery, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(arrived, hfsm::top_state()),
      HFSM_DEFINE_STATE_MAP(
          mc_state_map,
          FSM_STATE_MAP_ENTRY_EX_ALL(&start, nullptr, nullptr, nullptr),
          FSM_STATE_MAP_ENTRY_EX_ALL(&vector, nullptr, &entry_vector, nullptr),
          FSM_STATE_MAP_ENTRY_EX_ALL(&collision_avoidance,
                                     nullptr,
                                     &entry_collision_avoidance,
                                     &exit_collision_avoidance),
          FSM_STATE_MAP_ENTRY_EX_ALL(&collision_recovery,
                                     nullptr,
                                     &entry_collision_recovery,
                                     nullptr),
          FSM_STATE_MAP_ENTRY_EX_ALL(&new_direction,
                                     nullptr,
                                     &entry_new_direction,
                                     nullptr),
          FSM_STATE_MAP_ENTRY_EX_ALL(&arrived, nullptr, nullptr, nullptr)) {}

/*******************************************************************************
 * States
 ******************************************************************************/
RCSW_CONST FSM_STATE_DEFINE_ND(vector_fsm, start) {
  return util_signal::ekHANDLED;
}

FSM_STATE_DEFINE_ND(vector_fsm, collision_avoidance) {
  if (ekST_COLLISION_AVOIDANCE != last_state()) {
    ER_DEBUG("Executing ekST_COLLIISION_AVOIDANCE");
  }
  /*
   * If we came from the NEW_DIRECTION_STATE, then we got there from this
   * function, and have just finished changing our direction due to a frequent
   * collision. As such, we need to go into collision recovery, and zoom in our
   * new direction away from whatever is causing the problem. See #243.
   */
  if (ekST_NEW_DIRECTION == previous_state()) {
    rmath::vector2d force(
        actuation()->actuator<kin2D::governed_diff_drive>()->max_speed() * 0.7,
        rmath::radians(0.0));
    saa()->steer_force2D().accum(force);
    internal_event(ekST_COLLISION_RECOVERY);
    return util_signal::ekHANDLED;
  }

  if (auto obs =
          sensing()->sensor<hal::sensors::proximity_sensor>()->avg_prox_obj()) {
    ER_DEBUG("Found threatening obstacle: %s@%f [%f]",
             obs->to_str().c_str(),
             obs->angle().value(),
             obs->length());
    saa()->steer_force2D().accum(saa()->steer_force2D().avoidance(*obs));
    /*
     * If we are currently spinning in place (hard turn), we have 0 linear
     * velocity, and that does not play well with the arrival force
     * calculations. To fix this, add a bit of wander force.
     */
    if (saa()->linear_velocity().length() <= 0.1) {
      saa()->steer_force2D().accum(saa()->steer_force2D().wander(rng()));
    }
  } else {
    /*
     * Go in whatever direction you are currently facing for collision recovery.
     */
    rmath::vector2d force(
        actuation()->actuator<kin2D::governed_diff_drive>()->max_speed() * 0.7,
        rmath::radians(0.0));
    saa()->steer_force2D().accum(force);
    internal_event(ekST_COLLISION_RECOVERY);
  }
  return util_signal::ekHANDLED;
}

FSM_STATE_DEFINE_ND(vector_fsm, collision_recovery) {
  if (ekST_COLLISION_RECOVERY != last_state()) {
    ER_DEBUG("Executing ekST_COLLISION_RECOVERY");
  }

  /*
   * Even though we are recovering from our last collision, and there are
   * hopefully no obstacles nearby, we still need to check if another obstacle
   * threatens. Failure to do this can lead to situations where a robot is
   * "recovering" and moving directly towards a wall, and if the wall is
   * currently just out of range of their proximity sensor upon entering this
   * state, then *occasionally* the robot will end up moving inside of the wall,
   * and somehow the physics engine bounceback does not handle that correctly
   * (or maybe that is how it is supposed to work; idk). This causes an
   * exception and ARGoS crashes.
   */
  if (auto obs =
          sensing()->sensor<hal::sensors::proximity_sensor>()->avg_prox_obj()) {
    m_state.m_collision_rec_count = 0;
    internal_event(ekST_COLLISION_AVOIDANCE);
  } else if (++m_state.m_collision_rec_count >= kCOLLISION_RECOVERY_TIME) {
    m_state.m_collision_rec_count = 0;
    internal_event(ekST_VECTOR);
  }
  return util_signal::ekHANDLED;
}

FSM_STATE_DEFINE(vector_fsm, vector, rpfsm::event_data* data) {
  if (ekST_VECTOR != last_state()) {
    ER_DEBUG("Executing ekST_VECTOR");
  }

  auto* goal = dynamic_cast<const struct goal_data*>(data);
  if (nullptr != goal) {
    m_goal_data = *goal;
    ER_INFO("Target=%s, robot=%s",
            m_goal_data.loc.to_str().c_str(),
            saa()->sensing()->position().to_str().c_str());
  }

  if ((m_goal_data.loc - sensing()->position()).length() <=
      m_goal_data.tolerance) {
    internal_event(ekST_ARRIVED,
                   std::make_unique<struct goal_data>(m_goal_data));
  }

  /*
   * Only go into collision avoidance if we are not really close to our
   * target. If we are close, then ignore obstacles (the other guy will
   * move!). 'MURICA.
   *
   * Not doing this results in controller getting stuck when they all are trying to
   * acquire locations in close quarters.
   */
  if (saa()->sensing()->sensor<hal::sensors::proximity_sensor>()->avg_prox_obj() &&
      !saa()->steer_force2D().within_slowing_radius()) {
    internal_event(ekST_COLLISION_AVOIDANCE);
  } else {
    saa()->steer_force2D().accum(
        saa()->steer_force2D().seek_to(m_goal_data.loc));
    saa()->actuation()->actuator<hal::actuators::led_actuator>()->set_color(
        -1, rutils::color::kBLUE);
  }
  return util_signal::ekHANDLED;
}

RCSW_CONST FSM_STATE_DEFINE(vector_fsm,
                            arrived,
                            RCSW_UNUSED struct goal_data* data) {
  if (ekST_ARRIVED != last_state()) {
    ER_DEBUG("Executing ekST_ARRIVED: target=%s, tol=%f",
             data->loc.to_str().c_str(),
             data->tolerance);
  }
  return util_signal::ekHANDLED;
}

FSM_ENTRY_DEFINE_ND(vector_fsm, entry_vector) {
  ER_DEBUG("Entering ekST_VECTOR");
  actuation()->actuator<hal::actuators::led_actuator>()->set_color(
      -1, rutils::color::kBLUE);
}

FSM_ENTRY_DEFINE_ND(vector_fsm, entry_collision_avoidance) {
  ER_DEBUG("Entering ekST_COLLISION_AVOIDANCE");
  ca_tracker()->ca_enter();
  actuation()->actuator<hal::actuators::led_actuator>()->set_color(
      -1, rutils::color::kRED);
}

FSM_EXIT_DEFINE(vector_fsm, exit_collision_avoidance) {
  ER_DEBUG("Exiting ekST_COLLISION_AVOIDANCE");
  ca_tracker()->ca_exit();
}

FSM_ENTRY_DEFINE_ND(vector_fsm, entry_collision_recovery) {
  ER_DEBUG("Entering ekST_COLLISION_RECOVERY");
  actuation()->actuator<hal::actuators::led_actuator>()->set_color(
      -1, rutils::color::kYELLOW);
}

/*******************************************************************************
 * Collision Metrics
 ******************************************************************************/
bool vector_fsm::in_collision_avoidance(void) const {
  return ekST_COLLISION_AVOIDANCE == current_state();
} /* in_collision_avoidance() */

bool vector_fsm::entered_collision_avoidance(void) const {
  return ekST_COLLISION_AVOIDANCE != last_state() && in_collision_avoidance();
} /* entered_collision_avoidance() */

bool vector_fsm::exited_collision_avoidance(void) const {
  return ekST_COLLISION_AVOIDANCE == last_state() && !in_collision_avoidance();
} /* exited_collision_avoidance() */

rmath::vector2u vector_fsm::avoidance_loc(void) const {
  return saa()->sensing()->discrete_position();
} /* avoidance_loc() */

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
void vector_fsm::task_start(const ta::taskable_argument* const c_arg) {
  static const uint8_t kTRANSITIONS[] = {
      ekST_VECTOR,            /* start */
      ekST_VECTOR,            /* vector */
      util_signal::ekIGNORED, /* collision avoidance */
      util_signal::ekIGNORED, /* collision recovery */
      util_signal::ekIGNORED, /* new direction */
      util_signal::ekIGNORED, /* arrived */
  };
  auto* const a = dynamic_cast<const point_argument*>(c_arg);
  ER_ASSERT(nullptr != a, "Bad point argument passed to %s", __FUNCTION__);
  FSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()],
                 std::make_unique<struct goal_data>(a->point(), a->tolerance()));
} /* task_start() */

void vector_fsm::task_execute(void) {
  inject_event(util_signal::ekRUN, rpfsm::event_type::ekNORMAL);
} /* task_execute() */

void vector_fsm::init(void) {
  actuation()->reset();
  util_hfsm::init();
} /* init() */

rmath::vector2d vector_fsm::calc_vector_to_goal(const rmath::vector2d& goal) {
  return goal - sensing()->position();
} /* calc_vector_to_goal() */

NS_END(fsm, cosm);
