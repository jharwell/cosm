/**
 * \file util_hfsm.cpp
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
#include "cosm/spatial/fsm/util_hfsm.hpp"

#include <numeric>

#include "cosm/spatial/fsm/util_signal.hpp"
#include "cosm/subsystem/actuation_subsystem2D.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
util_hfsm::util_hfsm(csubsystem::saa_subsystemQ3D* const saa,
                     rmath::rng* rng,
                     uint8_t max_states)
    : rpfsm::hfsm(max_states),
      ER_CLIENT_INIT("cosm.spatial.fsm.util_hfsm"),
      RCPPSW_HFSM_CONSTRUCT_STATE(transport_to_nest, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(leaving_nest, hfsm::top_state()),
      m_saa(saa),
      m_tracker(sensing()),
      m_rng(rng) {}

/*******************************************************************************
 * States
 ******************************************************************************/

RCPPSW_HFSM_STATE_DEFINE(util_hfsm, leaving_nest, rpfsm::event_data* data) {
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
  auto* prox =
      m_saa->sensing()->template sensor<hal::sensors::proximity_sensor>();
  if (auto obs = prox->avg_prox_obj()) {
    m_tracker.inta_enter();
    m_saa->steer_force2D().accum(m_saa->steer_force2D().avoidance(*obs));
  } else {
    m_tracker.inta_exit();
  }
  m_saa->steer_force2D().accum(m_saa->steer_force2D().wander(m_rng));

  auto* ground =
      m_saa->sensing()->template sensor<hal::sensors::ground_sensor>();
  if (!ground->detect(hal::sensors::ground_sensor::kNestTarget)) {
    return util_signal::ekLEFT_NEST;
  }
  return rpfsm::event_signal::ekHANDLED;
} /* leaving_nest() */

RCPPSW_HFSM_STATE_DEFINE(util_hfsm, transport_to_nest, nest_transport_data* data) {
  ER_ASSERT(rpfsm::event_type::ekNORMAL == data->type(),
            "ekST_TRANSPORT_TO_NEST cannot handle child events");
  if (current_state() != last_state()) {
    ER_DEBUG("Executing ekST_TRANSPORT_TO_NEST");
  }
  event_data_hold(true);

  auto* ground =
      m_saa->sensing()->template sensor<hal::sensors::ground_sensor>();

  if (ground->detect(hal::sensors::ground_sensor::kNestTarget)) {
    auto dist_to_light = (m_saa->sensing()->rpos2D() - data->nest_loc).length();
    if (!m_nest_thresh) {
      auto dist = rtypes::spatial_dist(rng()->uniform(0.01, dist_to_light));
      m_nest_thresh = boost::make_optional(dist);
    }
    if (dist_to_light <= m_nest_thresh->v()) {
      /*
       * We have arrived at the nest so send this signal to the parent FSM that
       * is listing for it and stop moving.
       */
      actuation()->actuator<ckin2D::governed_diff_drive>()->reset();
      m_nest_thresh.reset();
      return util_signal::ekENTERED_NEST;
    }
  }

  /*
   * Add a bit of wander force when returning to the nest so that we do not
   * beeline for its center directly to decrease congestion.
   */
  auto* light = m_saa->sensing()->template sensor<hal::sensors::light_sensor>();
  /* m_saa->steer_force2D().accum(m_saa->steer_force2D().wander(m_rng)); */
  m_saa->steer_force2D().accum(
      m_saa->steer_force2D().phototaxis(light->readings()));

  auto* prox =
      m_saa->sensing()->template sensor<hal::sensors::proximity_sensor>();
  if (auto obs = prox->avg_prox_obj()) {
    m_tracker.inta_enter();
    m_saa->steer_force2D().accum(m_saa->steer_force2D().avoidance(*obs));
  } else {
    m_tracker.inta_exit();
  }

  return rpfsm::event_signal::ekHANDLED;
} /* transport_to_nest() */

RCPPSW_HFSM_ENTRY_DEFINE_ND(util_hfsm, entry_leaving_nest) {
  actuation()->template actuator<hal::actuators::led_actuator>()->set_color(
      -1, rutils::color::kWHITE);
}

RCPPSW_HFSM_ENTRY_DEFINE_ND(util_hfsm, entry_transport_to_nest) {
  sensing()->template sensor<hal::sensors::light_sensor>()->enable();
  actuation()->template actuator<hal::actuators::led_actuator>()->set_color(
      -1, rutils::color::kGREEN);
}

RCPPSW_HFSM_EXIT_DEFINE(util_hfsm, exit_transport_to_nest) {
  sensing()->template sensor<hal::sensors::light_sensor>()->disable();
}

RCPPSW_HFSM_ENTRY_DEFINE_ND(util_hfsm, entry_wait_for_signal) {
  actuation()->template actuator<kin2D::governed_diff_drive>()->reset();
  actuation()->template actuator<hal::actuators::led_actuator>()->set_color(
      -1, rutils::color::kWHITE);
}

/*******************************************************************************
 * General Member Functions
 ******************************************************************************/
rmath::radians util_hfsm::random_angle(void) {
  return rmath::radians(m_rng->uniform(0.0, rmath::radians::kPI.v()));
} /* randomize_vector_angle() */

csubsystem::sensing_subsystemQ3D* util_hfsm::sensing(void) {
  return m_saa->sensing();
}

const csubsystem::sensing_subsystemQ3D* util_hfsm::sensing(void) const {
  return m_saa->sensing();
}

csubsystem::actuation_subsystem2D* util_hfsm::actuation(void) {
  return m_saa->actuation();
}

csubsystem::actuation_subsystem2D* util_hfsm::actuation(void) const {
  return m_saa->actuation();
}

NS_END(fsm, spatial, cosm);
