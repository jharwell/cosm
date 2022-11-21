/**
 * \file vector_fsm.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/fsm/vector_fsm.hpp"

#include "cosm/spatial/fsm/util_signal.hpp"
#include "cosm/subsystem/actuation_subsystem.hpp"
#include "cosm/subsystem/base_saa_subsystem.hpp"
#include "cosm/subsystem/sensing_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::spatial::fsm {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
vector_fsm::vector_fsm(const csfsm::fsm_params* params, rmath::rng* rng)
    : util_hfsm(params, rng, ekST_MAX_STATES),
      ER_CLIENT_INIT("cosm.spatial.fsm.vector"),
      RCPPSW_HFSM_CONSTRUCT_STATE(start, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(vector, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(interference_avoidance, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(interference_recovery, hfsm::top_state()),
      RCPPSW_HFSM_CONSTRUCT_STATE(arrived, hfsm::top_state()),
      RCPPSW_HFSM_DEFINE_STATE_MAP(
          mc_state_map,
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&start, nullptr, nullptr, nullptr),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&vector,
                                             nullptr,
                                             &entry_vector,
                                             nullptr),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&interference_avoidance,
                                             nullptr,
                                             &entry_interference_avoidance,
                                             &exit_interference_avoidance),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&interference_recovery,
                                             nullptr,
                                             &entry_interference_recovery,
                                             nullptr),
          RCPPSW_HFSM_STATE_MAP_ENTRY_EX_ALL(&arrived,
                                             nullptr,
                                             nullptr,
                                             nullptr)) {}

/*******************************************************************************
 * States
 ******************************************************************************/
RCPPSW_CONST RCPPSW_HFSM_STATE_DEFINE_ND(vector_fsm, start) {
  return util_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE_ND(vector_fsm, interference_avoidance) {
  if (ekST_INTERFERENCE_AVOIDANCE != last_state()) {
    ER_DEBUG("Executing ekST_COLLIISION_AVOIDANCE");
  }

  if (auto obs =
          sensing()->sensor<hal::sensors::proximity_sensor>()->avg_prox_obj()) {
    ER_DEBUG("Found threatening obstacle: %s@%f [%f]",
             obs->to_str().c_str(),
             obs->angle().v(),
             obs->length());
    saa()->apf().accum(saa()->apf().avoidance(*obs));
    auto odom = saa()->sensing()->odometry()->reading();
  } else {
    internal_event(ekST_INTERFERENCE_RECOVERY);
  }
  return util_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE_ND(vector_fsm, interference_recovery) {
  if (ekST_INTERFERENCE_RECOVERY != last_state()) {
    ER_DEBUG("Executing ekST_INTERFERENCE_RECOVERY");
  }

  /*
   * Even though we are recovering from our last interference, and there are
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
    m_state.m_interference_rec_count = 0;
    internal_event(ekST_INTERFERENCE_AVOIDANCE);
  } else if (++m_state.m_interference_rec_count >= kINTERFERENCE_RECOVERY_TIME) {
    m_state.m_interference_rec_count = 0;
    internal_event(ekST_VECTOR);
  }
  /*
   * Go in whatever direction you are currently facing for interference
   * recovery. You have to do this each timestep because the accumulated force
   * is reset at the end of the robot's control loop.
   */
  rmath::vector2d force(actuation()->locomotion()->max_velocity() * 0.7,
                        rmath::radians(0.0));
  saa()->apf().accum(force);
  return util_signal::ekHANDLED;
}

RCPPSW_HFSM_STATE_DEFINE_ND(vector_fsm, vector) {
  if (ekST_VECTOR != last_state()) {
    ER_DEBUG("Executing ekST_VECTOR");
    ER_INFO("Target=%s, robot=%s",
            m_goal.point().to_str().c_str(),
            saa()->sensing()->rpos2D().to_str().c_str());
  }

  if ((m_goal.point() - sensing()->rpos2D()).length() <= m_goal.tolerance()) {
    internal_event(ekST_ARRIVED, std::make_unique<point_argument>(m_goal));
  }

  /*
   * Only go into interference avoidance if we are not really close to our
   * target. If we are close, then ignore obstacles (the other guy will
   * move!). 'MURICA.
   *
   * Not doing this results in agents getting stuck when they all are trying
   * to acquire locations in close quarters.
   */
  if (saa()->sensing()->sensor<hal::sensors::proximity_sensor>()->avg_prox_obj() &&
      !saa()->apf().within_slowing_radius()) {
    internal_event(ekST_INTERFERENCE_AVOIDANCE);
  } else {
    saa()->apf().accum(saa()->apf().seek_to(m_goal.point()));
    actuation()->diagnostics()->emit(chactuators::diagnostics::ekVECTOR_TO_GOAL);
  }
  return util_signal::ekHANDLED;
}

RCPPSW_CONST RCPPSW_HFSM_STATE_DEFINE(vector_fsm,
                                      arrived,
                                      RCPPSW_UNUSED point_argument* data) {
  if (ekST_ARRIVED != last_state()) {
    ER_DEBUG("Executing ekST_ARRIVED: target=%s, tol=%f",
             data->point().to_str().c_str(),
             data->tolerance());
  }
  return util_signal::ekHANDLED;
}

RCPPSW_HFSM_ENTRY_DEFINE_ND(vector_fsm, entry_vector) {
  ER_DEBUG("Entering ekST_VECTOR");
  actuation()->diagnostics()->emit(chactuators::diagnostics::ekVECTOR_TO_GOAL);
}

RCPPSW_HFSM_ENTRY_DEFINE_ND(vector_fsm, entry_interference_avoidance) {
  ER_DEBUG("Entering ekST_INTERFERENCE_AVOIDANCE");
  inta_tracker()->state_enter();
  actuation()->diagnostics()->emit(chactuators::diagnostics::ekEXP_INTERFERENCE);
}

RCPPSW_HFSM_EXIT_DEFINE(vector_fsm, exit_interference_avoidance) {
  ER_DEBUG("Exiting ekST_INTERFERENCE_AVOIDANCE");
  inta_tracker()->state_exit();
}

RCPPSW_HFSM_ENTRY_DEFINE_ND(vector_fsm, entry_interference_recovery) {
  ER_DEBUG("Entering ekST_INTERFERENCE_RECOVERY");
  actuation()->diagnostics()->emit(chactuators::diagnostics::ekEXP_INTERFERENCE);
}

/*******************************************************************************
 * Interference Metrics
 ******************************************************************************/
bool vector_fsm::exp_interference(void) const {
  return ekST_INTERFERENCE_AVOIDANCE == current_state();
} /* exp_interference() */

bool vector_fsm::entered_interference(void) const {
  return ekST_INTERFERENCE_AVOIDANCE != last_state() && exp_interference();
} /* entered_interference() */

bool vector_fsm::exited_interference(void) const {
  return ekST_INTERFERENCE_AVOIDANCE == last_state() && !exp_interference();
} /* exited_interference() */

boost::optional<rmath::vector3z> vector_fsm::interference_loc3D(void) const {
  if (exp_interference()) {
    return boost::make_optional(saa()->sensing()->dpos3D());
  }
  return boost::none;
} /* interference_loc3D() */

/*******************************************************************************
 * Taskable Interface
 ******************************************************************************/
void vector_fsm::task_start(ta::taskable_argument* c_arg) {
  static const uint8_t kTRANSITIONS[] = {
    ekST_VECTOR, /* start */
    ekST_VECTOR, /* vector */
    util_signal::ekIGNORED, /* interference avoidance */
    util_signal::ekIGNORED, /* interference recovery */
    util_signal::ekIGNORED, /* arrived */
  };
  auto* const a = dynamic_cast<const point_argument*>(c_arg);
  m_goal = std::move(*a);

  ER_ASSERT(nullptr != a, "Bad point argument passed to %s", __FUNCTION__);
  RCPPSW_HFSM_VERIFY_TRANSITION_MAP(kTRANSITIONS, ekST_MAX_STATES);
  external_event(kTRANSITIONS[current_state()],
                 std::make_unique<point_argument>(a->tolerance(), a->point()));
} /* task_start() */

void vector_fsm::task_execute(void) {
  inject_event(util_signal::ekRUN, rpfsm::event_type::ekNORMAL);
} /* task_execute() */

void vector_fsm::task_reset(void) { init(); } /* task_reset() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::vector2d vector_fsm::calc_vector_to_goal(const rmath::vector2d& goal) {
  return goal - sensing()->rpos2D();
} /* calc_vector_to_goal() */

void vector_fsm::init(void) {
  actuation()->reset();
  util_hfsm::init();
} /* init() */

} /* namespace cosm::spatial::fsm */
