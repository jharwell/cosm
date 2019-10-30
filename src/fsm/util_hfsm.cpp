/**
 * @file util_hfsm.cpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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
#include "cosm/fsm/util_hfsm.hpp"

#include "cosm/fsm/new_direction_data.hpp"
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
util_hfsm::util_hfsm(subsystem::saa_subsystem2D* const saa,
                     rmath::rng* rng,
                     uint8_t max_states)
    : rpfsm::hfsm(max_states),
      ER_CLIENT_INIT("cosm.fsm.util_hfsm"),
      HFSM_CONSTRUCT_STATE(transport_to_nest, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(leaving_nest, hfsm::top_state()),
      HFSM_CONSTRUCT_STATE(new_direction, hfsm::top_state()),
      m_saa(saa),
      m_tracker(sensing()),
      m_rng(rng) {}

/*******************************************************************************
 * States
 ******************************************************************************/
HFSM_STATE_DEFINE(util_hfsm, leaving_nest, rpfsm::event_data* data) {
  ER_ASSERT(rpfsm::event_type::ekNORMAL == data->type(),
            "ekST_LEAVING_NEST cannot handle child events");

  if (current_state() != last_state()) {
    ER_DEBUG("Executing ekST_LEAVING_NEST");
  }
  /*
   * We don't want to just apply anti-phototaxis force, because that will make
   * the robot immediately turn around as soon as it has entered the nest and
   * dropped its block, leading to a lot of traffic jams by the edge of the
   * nest. Instead, wander about within the nest until you find the edge (either
   * on your own or being pushed out via collision avoidance).
   */
  if (auto obs = m_saa->sensing()
                     ->sensor<hal::sensors::proximity_sensor>()
                     ->avg_prox_obj()) {
    m_tracker.ca_enter();
    m_saa->steer_force2D().accum(m_saa->steer_force2D().avoidance(*obs));
  } else {
    m_tracker.ca_exit();
  }
  m_saa->steer_force2D().accum(m_saa->steer_force2D().wander(m_rng));

  if (!m_saa->sensing()->sensor<hal::sensors::ground_sensor>()->detect(
          hal::sensors::ground_sensor::kNestTarget)) {
    return util_signal::ekLEFT_NEST;
  }
  return rpfsm::event_signal::ekHANDLED;
} /* leaving_nest() */

HFSM_STATE_DEFINE(util_hfsm, transport_to_nest, rpfsm::event_data* data) {
  ER_ASSERT(rpfsm::event_type::ekNORMAL == data->type(),
            "ekST_TRANSPORT_TO_NEST cannot handle child events");
  if (current_state() != last_state()) {
    ER_DEBUG("Executing ekST_TRANSPORT_TO_NEST");
  }

  /*
   * We have arrived at the nest so send this signal to the parent FSM that is
   * listing for it.
   */
  if (m_saa->sensing()->sensor<hal::sensors::ground_sensor>()->detect(
          hal::sensors::ground_sensor::kNestTarget)) {
    if (m_nest_count++ < kNEST_COUNT_MAX_STEPS) {
      m_saa->steer_force2D().accum(m_saa->steer_force2D().wander(m_rng));
      return util_signal::ekHANDLED;
    } else {
      m_nest_count = 0;
      return util_signal::ekENTERED_NEST;
    }
  }

  /*
   * Add a bit of wander force when returning to the nest so that we do not
   * beeline for its center directly to decrease congestion.
   */
  m_saa->steer_force2D().accum(m_saa->steer_force2D().wander(m_rng));
  m_saa->steer_force2D().accum(m_saa->steer_force2D().phototaxis(
      m_saa->sensing()->sensor<hal::sensors::light_sensor>()->readings()));

  if (auto obs = m_saa->sensing()
                     ->sensor<hal::sensors::proximity_sensor>()
                     ->avg_prox_obj()) {
    m_tracker.ca_enter();
    m_saa->steer_force2D().accum(m_saa->steer_force2D().avoidance(*obs));
  } else {
    m_tracker.ca_exit();
  }

  return rpfsm::event_signal::ekHANDLED;
} /* transport_to_nest() */

HFSM_STATE_DEFINE(util_hfsm, new_direction, rpfsm::event_data* data) {
  rmath::radians current_dir = m_saa->sensing()->heading();

  /*
   * The new direction is only passed the first time this state is entered, so
   * save it. After that, a standard HFSM signal is passed we which ignore.
   */
  auto* dir_data = dynamic_cast<const new_direction_data*>(data);
  if (nullptr != dir_data) {
    m_new_dir = dir_data->dir;
    m_new_dir_count = 0;
    ER_DEBUG("Change direction: %f -> %f",
             current_dir.value(),
             m_new_dir.value());
  }

  /*
   * The amount we change our direction is proportional to how far off we are
   * from our desired new direction. This prevents excessive spinning due to
   * overshoot. We reset the steering force before applying our direction change
   * in order to ensure we are ONLY changing our direction (it is possible we
   * had already accumed some forces before the calculation that resulted in a
   * direction change command was performed).
   */
  rmath::vector2d dir_change(
      actuation()->actuator<kin2D::governed_diff_drive>()->max_speed() * 0.1,
      current_dir - m_new_dir);
  m_saa->steer_force2D().accum(dir_change);

  /*
   * We limit the maximum # of steps that we spin, and have an arrival tolerance
   * to also help limit excessive spinning. See #191.
   */
  if (std::fabs((current_dir - m_new_dir).value()) < kDIR_CHANGE_TOL ||
      m_new_dir_count >= kDIR_CHANGE_MAX_STEPS) {
    internal_event(previous_state());
  }
  ++m_new_dir_count;
  return util_signal::ekHANDLED;
}

HFSM_ENTRY_DEFINE_ND(util_hfsm, entry_leaving_nest) {
  actuation()->actuator<hal::actuators::led_actuator>()->set_color(
      -1, rutils::color::kWHITE);
}
HFSM_ENTRY_DEFINE_ND(util_hfsm, entry_transport_to_nest) {
  sensing()->sensor<hal::sensors::light_sensor>()->enable();
  actuation()->actuator<hal::actuators::led_actuator>()->set_color(
      -1, rutils::color::kGREEN);
}
HFSM_EXIT_DEFINE(util_hfsm, exit_transport_to_nest) {
  sensing()->sensor<hal::sensors::light_sensor>()->disable();
}
HFSM_ENTRY_DEFINE_ND(util_hfsm, entry_new_direction) {
  actuation()->actuator<hal::actuators::led_actuator>()->set_color(
      -1, rutils::color::kCYAN);
}
HFSM_ENTRY_DEFINE_ND(util_hfsm, entry_wait_for_signal) {
  actuation()->actuator<kin2D::governed_diff_drive>()->reset();
  m_saa->actuation()->actuator<hal::actuators::led_actuator>()->set_color(
      -1, rutils::color::kWHITE);
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
rmath::radians util_hfsm::random_angle(void) {
  return rmath::radians(m_rng->uniform(0.0, 1.0));
} /* randomize_vector_angle() */

subsystem::sensing_subsystem2D* util_hfsm::sensing(void) {
  return saa()->sensing();
}

const subsystem::sensing_subsystem2D* util_hfsm::sensing(void) const {
  return saa()->sensing();
}

subsystem::actuation_subsystem2D* util_hfsm::actuation(void) {
  return saa()->actuation();
}

const subsystem::actuation_subsystem2D* util_hfsm::actuation(void) const {
  return saa()->actuation();
}

NS_END(fsm, cosm);
