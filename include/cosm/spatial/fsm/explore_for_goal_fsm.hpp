/**
 * \file explore_for_goal_fsm.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <functional>
#include <memory>

#include "cosm/cosm.hpp"
#include "cosm/spatial/strategy/explore/base_explore.hpp"
#include "cosm/spatial/metrics/interference_metrics.hpp"
#include "cosm/spatial/fsm/util_hfsm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::spatial::fsm {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class explore_for_goal_fsm
 * \ingroup spatial fsm
 *
 * \brief Robots executing this task will execute a specified exploration
 * behavior while looking for an instance of their goal. Once they have found
 * one, the FSM will signal that its task is complete.
 *
 * It is also possible to run this FSM with NO exploration behavior, as this can
 * be necessary for higher level FSMs that require acquisition of goals via
 * vectoring, and the general purpose machinery in \ref acquire_goal_fsm always
 * falls back on this FSM when no known candidates of the goal type are
 * currently known.
 */
class explore_for_goal_fsm final : public csfsm::util_hfsm,
                                   public cta::taskable,
                                   public rer::client<explore_for_goal_fsm> {
 public:
  enum state {
    /**
     * Transient starting state
     */
    ekST_START,

    /**
     * Roaming around looking for a goal.
     */
    ekST_EXPLORE,

    /**
     * A goal has been acquired.
     */
    ekST_FINISHED,
    ekST_MAX_STATES
  };

  explore_for_goal_fsm(const csfsm::fsm_params* params,
                       std::unique_ptr<cssexplore::base_explore> behavior,
                       rmath::rng* rng,
                       const std::function<bool(void)>& goal_detect);
  ~explore_for_goal_fsm(void) override = default;
  explore_for_goal_fsm& operator=(const explore_for_goal_fsm&) = delete;
  explore_for_goal_fsm(const explore_for_goal_fsm&) = delete;

  /* taskable overrides */
  bool task_finished(void) const override RCPPSW_PURE;
  void task_start(ta::taskable_argument* c_arg) override;
  bool task_running(void) const override RCPPSW_PURE;
  void task_reset(void) override;
  void task_execute(void) override;

 private:
  /* exploration states */

  /**
   * \brief Starting/reset state for FSM. Has no purpose other than that.
   */
  RCPPSW_HFSM_STATE_DECLARE_ND(explore_for_goal_fsm, start);

  /**
   * \brief The main state for the explore FSM. Robots in this state maintain
   * their heading, looking for SOMETHING of interest, until they find it or
   * exceed the direction change threshold.
   */
  RCPPSW_HFSM_STATE_DECLARE_ND(explore_for_goal_fsm, explore);

  /**
   * \brief Once a goal has been acquired, controller wait in this state until
   * reset by a higher level FSM.
   */
  RCPPSW_HFSM_STATE_DECLARE_ND(explore_for_goal_fsm, finished);

  /**
   * \brief Simple state for entry in the main exploration state, used to change
   * LED color for visualization purposes.
   */
  RCPPSW_HFSM_ENTRY_DECLARE_ND(explore_for_goal_fsm, entry_explore);

  /**
   * \brief Defines the state map for the FSM.
   *
   * Note that the order of the states in the map MUST match the order of the
   * states in \enum state, or things will not work correctly.
   */
  RCPPSW_HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
    return &mc_state_map[index];
  }

  RCPPSW_HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ekST_MAX_STATES);

  /* clang-format off */
  std::unique_ptr<cssexplore::base_explore>  m_behavior;
  std::function<bool(void)>                  m_goal_detect;
  /* clang-format on */
};

} /* namespace cosm::spatial::fsm */
