/**
 * \file supervisor_fsm.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>
#include <variant>

#include "rcppsw/patterns/fsm/simple_fsm.hpp"

#include "cosm/cosm.hpp"
#include "cosm/ta/taskable.hpp"
#include "cosm/subsystem/subsystem_fwd.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ta {
class base_executive;
} /* namespace ta */

namespace cosm::fsm {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class supervisor_fsm
 * \ingroup fsm
 *
 * \brief An FSM used supervise robot operation.
 *
 * In normal operation, the \ref cta::taskable object is just executed. If a
 * non-standard event is received (implemented in derived classes), then
 * normal operation can be replaced by something else (e.g., stopping all robot
 * motion if a robot malfunction event is received).
 */
class supervisor_fsm final : public rpfsm::simple_fsm,
                             public rer::client<supervisor_fsm> {
  using supervisee_variant_type =
      std::variant<ta::taskable*, ta::base_executive*>;

 public:
  explicit supervisor_fsm(csubsystem::base_saa_subsystem* saa);

  supervisor_fsm& operator=(const supervisor_fsm&) = delete;
  supervisor_fsm(const supervisor_fsm&) = delete;

  bool state_is_stopped(void) const {
    return current_state() == states::ekST_STOP;
  }

  /**
   * \brief Signal that the  \ref cta::taskable object should not be run every
   * timestep until the robot has been repaired.
   */
  void event_malfunction(void);

  /**
   * \brief Signal that the  \ref cta::taskable object should not be run every
   * timestep until the robot has been repaired.
   */
  void event_repair(void);

  /**
   * \brief Signal that the \ref cta::taskable object should not be run again until
   * the robot is reset, AND that all sensors and actuators should be disabled.
   */
  void event_stop(void);

  template <typename T>
  void supervisee_update(T* h) {
    m_supervisee = supervisee_variant_type(h);
  }

  void run(void) {
    inject_event(rpfsm::event_signal::ekRUN, rpfsm::event_type::ekNORMAL);
  }

 private:
  enum states {
    ekST_START,
    /**
     * Normal operation.
     */
    ekST_NORMAL,

    /**
     * Non-normal operation: the robot has malfunctioned.
     */
    ekST_MALFUNCTION,

    /**
     * Non-normal operation: the robot has ceased all further operations.
     */
    ekST_STOP,

    ekST_MAX_STATES
  };

  /* supervisor states */
  RCPPSW_FSM_STATE_DECLARE_ND(supervisor_fsm, start);
  RCPPSW_FSM_STATE_DECLARE_ND(supervisor_fsm, normal);
  RCPPSW_FSM_STATE_DECLARE_ND(supervisor_fsm, malfunction);
  RCPPSW_FSM_ENTRY_DECLARE_ND(supervisor_fsm, entry_stop);
  RCPPSW_FSM_STATE_DECLARE_ND(supervisor_fsm, stop);

  RCPPSW_FSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
    return &mc_state_map[index];
  }

  RCPPSW_FSM_DECLARE_STATE_MAP(state_map_ex,
                               mc_state_map,
                               states::ekST_MAX_STATES);

  /* clang-format off */
  csubsystem::base_saa_subsystem* m_saa;
  supervisee_variant_type         m_supervisee{};
  /* clang-format on */
};

} /* namespace cosm::fsm */
