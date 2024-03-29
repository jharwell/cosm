/**
 * \file state_tracker.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/cosm.hpp"
#include "cosm/subsystem/subsystem_fwd.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::fsm {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class state_tracker
 * \ingroup fsm
 *
 * \brief Utility class for tracking when a robot enters/exits a state of some
 * kind (e.g., collision avoidance), and the time spent in that state. Not
 * limited to FSM states. Used to have the logic to track state enter and exit
 * in a single place.
 */
class state_tracker {
 public:
  explicit state_tracker(const csubsystem::sensing_subsystem* const sensing)
      : mc_sensing(sensing) {}
  virtual ~state_tracker(void) = default;

  state_tracker(const state_tracker&) = delete;
  state_tracker& operator=(const state_tracker&) = delete;

  /* state metrics */
  bool in_state(void) const RCPPSW_PURE;
  bool entered_state(void) const RCPPSW_PURE;
  bool exited_state(void) const RCPPSW_PURE;
  rtypes::timestep state_duration(void) const;
  rtypes::timestep state_entry_time(void) const;

  /**
   * \brief Get the robot's location. Useful for capturing loction on state
   * entry/exit.
   */
  rmath::vector3z state_loc3D(void) const RCPPSW_PURE;

  /**
   * \brief Handle all logic for entering the state; classes should only have to
   * call this function whenever the condition for triggering entry into the
   * state is detected. If the robot is already in the state, nothing happens.
   */
  void state_enter(void);

  /**
   * \brief Handle all logic for exiting the state; classes should
   * only have to call this function whenever the trigger conditions for being
   * in the state no longer exist/the condition for leaving the state is triggered.
   */
  void state_exit(void);

  /**
   * \brief Resets the state tracker to "not in state" regardless of what state
   * it is currently in.
   */
  void state_reset(void);

 private:
  /* clang-format off */
  const subsystem::sensing_subsystem* const mc_sensing;

  bool                                         m_entered_state{false};
  bool                                         m_exited_state{false};
  bool                                         m_in_state{false};
  rtypes::timestep                             m_state_start{0};
  /* clang-format on */
};

} /* namespace cosm::fsm */
