/**
 * \file foraging_util_hfsm.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include "cosm/foraging/fsm/foraging_util_hfsm.hpp"

#include <numeric>

#include "cosm/spatial/fsm/point_argument.hpp"
#include "cosm/spatial/fsm/util_signal.hpp"
#include "cosm/subsystem/actuation_subsystem2D.hpp"
#include "cosm/subsystem/saa_subsystemQ3D.hpp"
#include "cosm/subsystem/sensing_subsystemQ3D.hpp"
#include "cosm/spatial/strategy/nest_acq/base_nest_acq.hpp"
#include "cosm/spatial/strategy/blocks/drop/base_drop.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, fsm);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
foraging_util_hfsm::foraging_util_hfsm(
    const csfsm::fsm_params* params,
    std::unique_ptr<cssnest_acq::base_nest_acq> nest_acq,
    std::unique_ptr<cssblocks::drop::base_drop> block_drop,
    rmath::rng* rng,
    uint8_t max_states)
    : util_hfsm(params, rng, max_states),
      ER_CLIENT_INIT("cosm.foraging.fsm.foraging_util_hfsm"),
      RCPPSW_HFSM_CONSTRUCT_STATE(transport_to_nest, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(drop_carried_block, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(leaving_nest, hfsm::top_state()),
      m_nest_acq(std::move(nest_acq)),
      m_block_drop(std::move(block_drop)) {}

/*******************************************************************************
 * States
 ******************************************************************************/
RCPPSW_HFSM_STATE_DEFINE(foraging_util_hfsm,
                         leaving_nest,
                         rpfsm::event_data* data) {
  ER_ASSERT(rpfsm::event_type::ekNORMAL == data->type(),
            "ekST_LEAVING_NEST cannot handle child events");

  if (current_state() != last_state()) {
    ER_DEBUG("Executing ekST_LEAVING_NEST");
    saa()->sensing()->env()->disable(chsensors::env_sensor::kBlockTarget);
  }
  inta_state_update();
  nz_state_update();

  /*
   * We don't want to just apply anti-phototaxis force, because that will make
   * the robot immediately turn around as soon as it has entered the nest and
   * dropped its block, leading to a lot of traffic jams by the edge of the
   * nest. Instead, wander about within the nest until you find the edge (either
   * on your own or being pushed out via collision avoidance).
   */
  /* saa()->steer_force2D().accum(saa()->steer_force2D().wander(rng())); */
  auto readings = saa()->sensing()->light()->readings();
  saa()->steer_force2D().accum(saa()->steer_force2D().anti_phototaxis(readings));

  /* don't use exited_nest() here--only high for a single timestep */
  auto env = saa()->sensing()->env();
  if (!env->detect(chsensors::env_sensor::kNestTarget)) {
    saa()->sensing()->env()->enable(chsensors::env_sensor::kBlockTarget);
    return csfsm::util_signal::ekLEFT_NEST;
  }
  return rpfsm::event_signal::ekHANDLED;
} /* leaving_nest() */

RCPPSW_HFSM_STATE_DEFINE(foraging_util_hfsm,
                         transport_to_nest,
                         nest_transport_data* data) {
  ER_ASSERT(rpfsm::event_type::ekNORMAL == data->type(),
            "ekST_TRANSPORT_TO_NEST cannot handle child events");
  if (current_state() != last_state()) {
    ER_DEBUG("Executing ekST_TRANSPORT_TO_NEST");
    ER_ASSERT(nullptr != m_nest_acq, "NULL nest acquisition behavior");
  }
  event_data_hold(true);

  if (!m_nest_acq->task_running()) {
    auto env = saa()->sensing()->env();

    /* We have entered the nest, so perform our acquisition strategy */
    if (env->detect(chsensors::env_sensor::kNestTarget)) {
      ER_DEBUG("Start nest acquisition strategy");
      /* tolerance not used at this level */
      csfsm::point_argument arg(-1, data->nest_loc);
      m_nest_acq->task_reset();
      m_nest_acq->task_start(&arg);
    }
  }

  if (m_nest_acq->task_running()) {
    ER_DEBUG("Nest acquisition strategy running");
    m_nest_acq->task_execute();
    if (m_nest_acq->task_finished()) {
      ER_DEBUG("Nest acquisition strategy finished");
      auto env = saa()->sensing()->env();
      if (env->detect(chsensors::env_sensor::kNestTarget)) {
        /*
         * We have arrived at the nest so stop moving and signal. Note we may
         * actually have arrived in the nest some time ago, but depending on our
         * acquisition strategy we don't signal until the strategy has
         * completed, because signaling triggers the block drop process.
         */
        saa()->actuation()->governed_diff_drive()->reset();
        event_data_hold(false);
        nz_state_update();
        return csfsm::util_signal::ekENTERED_NEST;
      } else { /* we are somehow outside the nest--try again */
        ER_DEBUG("Outside nest: Nest acquisition strategy reset");
        m_nest_acq->task_reset();
      }
    }
  } else { /* still outside the nest, just phototaxis + collision avoidance */
    ER_DEBUG("Outside nest");
    inta_state_update();
    auto readings = saa()->sensing()->light()->readings();
    saa()->steer_force2D().accum(saa()->steer_force2D().phototaxis(readings));
  }
  return rpfsm::event_signal::ekHANDLED;
} /* transport_to_nest() */

RCPPSW_HFSM_STATE_DEFINE(foraging_util_hfsm,
                         drop_carried_block,
                         rpfsm::event_data* data) {
  ER_ASSERT(rpfsm::event_type::ekNORMAL == data->type(),
            "ekST_DROP_CARRIED_BLOCK cannot handle child events");
  if (current_state() != last_state()) {
    ER_DEBUG("Executing ekST_DROP_CARRIED_BLOCK");
  }

  /* block drop behaviors are optional */
  if (nullptr == m_block_drop) {
    ER_DEBUG("Dropping block: no behavior");
    return csfsm::util_signal::ekDROPPED_BLOCK;
  }

  if (!m_block_drop->task_running()) {
    m_block_drop->task_reset();
    m_block_drop->task_start(nullptr);
  }

  if (m_block_drop->task_running()) {
    m_block_drop->task_execute();
    if (m_block_drop->task_finished()) {
      return csfsm::util_signal::ekDROPPED_BLOCK;
    }
  }
  return rpfsm::event_signal::ekHANDLED;
} /* transport_to_nest() */

RCPPSW_HFSM_ENTRY_DEFINE_ND(foraging_util_hfsm, entry_leaving_nest) {
  actuation()->diagnostics()->emit(chactuators::diagnostics::ekLEAVING_NEST);
}

RCPPSW_HFSM_ENTRY_DEFINE_ND(foraging_util_hfsm, entry_transport_to_nest) {
  sensing()->light()->enable();
  actuation()->diagnostics()->emit(chactuators::diagnostics::ekSUCCESS);
}

RCPPSW_HFSM_EXIT_DEFINE(foraging_util_hfsm, exit_transport_to_nest) {
  /* sensing()->light()->disable(); */
}

/*******************************************************************************
 * Nest Acquisition Metrics
 ******************************************************************************/
RCPPSW_WRAP_DEFP_OVERRIDE(foraging_util_hfsm,
                          nest_acq_strategy,
                          m_nest_acq,
                          nullptr,
                          const);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void foraging_util_hfsm::inta_state_update(void) {
  if (auto obs = saa()->sensing()->proximity()->avg_prox_obj()) {
    inta_tracker()->state_enter();
    saa()->steer_force2D().accum(saa()->steer_force2D().avoidance(*obs));
  } else {
    inta_tracker()->state_exit();
  }
} /* inta_state_update() */

void foraging_util_hfsm::nz_state_update(void) {
  auto env = saa()->sensing()->env();

  if (env->detect(chsensors::env_sensor::kNestTarget)) {
    nz_tracker()->state_enter();
  } else {
    nz_tracker()->state_exit();
  }
} /* nz_state_update() */

NS_END(fsm, foraging, cosm);
