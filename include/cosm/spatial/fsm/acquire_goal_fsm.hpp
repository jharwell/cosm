/**
 * \file acquire_goal_fsm.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_SPATIAL_FSM_ACQUIRE_GOAL_FSM_HPP_
#define INCLUDE_COSM_SPATIAL_FSM_ACQUIRE_GOAL_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>
#include <functional>
#include <list>
#include <memory>
#include <tuple>

#include "rcppsw/math/vector2.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/cosm.hpp"
#include "cosm/spatial/fsm/explore_for_goal_fsm.hpp"
#include "cosm/spatial/metrics/goal_acq_metrics.hpp"
#include "cosm/spatial/fsm/util_hfsm.hpp"
#include "cosm/spatial/fsm/vector_fsm.hpp"
#include "cosm/ta/taskable.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class acquire_goal_fsm
 * \ingroup spatial fsm
 *
 * \brief The base FSM for an acquiring a goal in the arena.
 *
 * Each robot executing this FSM will acquire an instance of its goal (either a
 * known instance or via random exploration). Once the instance has been
 * acquired, it signals that it has completed its task.
 */
class acquire_goal_fsm : public csfsm::util_hfsm,
                         public rer::client<acquire_goal_fsm>,
                         public cta::taskable,
                         public csmetrics::goal_acq_metrics {
 public:
  /**
   * \brief Tuple representing a goal to be acquired: A location, the utility
   * associated with that location, and an integer identifier for the location
   * (e.g. UUID of the object), which can be \ref rtypes::constants::kNoUUID if
   * unused.
   */
  using candidate_type = std::tuple<rmath::vector2d, double, rtypes::type_uuid>;

  /**
   * \brief Function type returning a \ref candidate_type if any is found, and
   * an empty boost::optional<> otherwise.
   */
  using goal_select_ftype = std::function<boost::optional<candidate_type>(void)>;

  /**
   * \brief Function type which returns the FSM's current goal via \ref
   * goal_acq_metrics::goal_type.
   */
  using acquisition_goal_ftype =
      std::function<goal_acq_metrics::goal_type(void)>;

  /**
   * \brief Function type which returns if the current acquisition goal is still
   * valid for acquisition, given its location and the ID of the goal. This is
   * necessary as the state of the arena may change between when a goal is
   * selected for acqusition and when a robot arrives, and we don't want to
   * waste effort attempting to acquire a goal that no longer exists (for
   * example).
   */
  using goal_valid_ftype =
      std::function<bool(const rmath::vector2d&, const rtypes::type_uuid&)>;

  struct hook_list {
    /* clang-format off */
    /**
     * \brief Callback used to tell the FSM what is the ultimate goal of the
     * acquisition. However, the return of \ref acquisition_goal() may not
     * always be the same as the specified goal, depending on what the current
     * FSM state is.
     */
    acquisition_goal_ftype    acquisition_goal{nullptr};

    /**
     * \brief Function used to select a goal from the list of candidates. Should
     * return the "best" candidate that should be acquired.
     */
    goal_select_ftype         goal_select{nullptr};

    /**
     * \brief Callback used to determine if any goal candidates are
     * currently available/eligible for acquisition.
     */
    std::function<bool(void)> candidates_exist{nullptr};

    /**
     * \brief Callback called on the first timestep of goal acquisition, in case
     * state needs to be reset after goal selection.
     */
    std::function<void(void)> begin_acq_cb{nullptr};

    /**
     * \brief Callback used after a goal has been acquired for sanity
     * check/verification of state. Will be passed \c TRUE if the acquired goal
     * was obtained via exploration, rather than vectoring, and false if it was
     * obtained via vectoring. Should return \c TRUE if the goal has REALLY been
     * acquired, and \c FALSE otherwise (the goal may have vanished if it was a
     * block/cache, for example).
     */
    std::function<bool(bool)> goal_acquired_cb{nullptr};

    /**
    * \brief Callback for goal detection during exploration. This fsm can't know
    * when a goal has been reached without losing its generality when exploring,
    * so this callback is provided to it for that purpose. Should return \c TRUE
    * if the goal has been detected/reached and exploration should terminate,
    * and \c FALSE otherwise.
    */
    std::function<bool(void)> explore_term_cb{nullptr};

    /**
     * \brief Callback for verifying goal validity during
     * vectoring/exploration. If for any reason the specific goal becomes
     * invalid before the robot has acquired it, then it should return \c FALSE.
     */
     goal_valid_ftype goal_valid_cb{nullptr};
    /* clang-format on */
  };

  /**
   * \param saa Handle to sensing and actuation subsystem.
   *
   * \param behavior The exploration behavior to use; can be NULL if goal
   * acquisition should always be performed via vectoring.
   *
   * \param rng Random number generator for use internally.
   *
   * \param hooks List of function callbacks that derived classes should pass in
   *              ordr to make use of the general purpose machinery in this
   *              class.
   */
  acquire_goal_fsm(const csfsm::fsm_params* params,
                   std::unique_ptr<csstrategy::base_strategy> behavior,
                   rmath::rng* rng,
                   const struct hook_list& hooks);
  ~acquire_goal_fsm(void) override = default;

  acquire_goal_fsm(const acquire_goal_fsm& fsm) = delete;
  acquire_goal_fsm& operator=(const acquire_goal_fsm& fsm) = delete;

  /* taskable overrides */
  void task_execute(void) override final;
  void task_start(ta::taskable_argument*) override {}
  bool task_finished(void) const override final {
    return ekST_FINISHED == current_state();
  }
  bool task_running(void) const override final {
    return ekST_ACQUIRE_GOAL == current_state();
  }
  void task_reset(void) override final { init(); }

  /* collision metrics */
  bool exp_interference(void) const override final RCPPSW_PURE;
  bool entered_interference(void) const override final RCPPSW_PURE;
  bool exited_interference(void) const override final RCPPSW_PURE;
  rtypes::timestep interference_duration(void) const override final RCPPSW_PURE;
  rmath::vector3z interference_loc3D(void) const override final RCPPSW_PURE;

  /* goal acquisition metrics */
  exp_status is_exploring_for_goal(void) const override final RCPPSW_PURE;
  bool is_vectoring_to_goal(void) const override final RCPPSW_PURE;
  bool goal_acquired(void) const override final RCPPSW_PURE;
  metrics::goal_acq_metrics::goal_type acquisition_goal(void) const override final;
  rmath::vector3z acquisition_loc3D(void) const override final RCPPSW_PURE;
  rmath::vector3z explore_loc3D(void) const override final RCPPSW_PURE;
  rmath::vector3z vector_loc3D(void) const override final RCPPSW_PURE;
  rtypes::type_uuid entity_acquired_id(void) const override final {
    return m_acq_id;
  }

  void init(void) override final;

 protected:
  enum states {
    /**
     * Transient starting state.
     */
    ekST_START,

    /**
     * Superstate for acquiring a goal via vectoring or exploration.
     */
    ekST_ACQUIRE_GOAL,

    /**
     * Goal has been acquired!
     */
    ekST_FINISHED,
    ekST_MAX_STATES
  };

 private:
  /**
   * \brief Acquire a known goal or discover one via random exploration.
   *
   * \return \c TRUE if a goal has been acquired, \c FALSE otherwise.
   */
  bool acquire_goal(void);

  /**
   * \brief Acquire an unknown goal via exploration.
   *
   * \return \c TRUE if a goal has been acquired \c FALSE otherwise.
   */
  bool acquire_unknown_goal(void);

  /**
   * \brief Acquire a known goal via exploration.
   *
   * \return \c TRUE if a goal has been acquired \c FALSE otherwise.
   */
  bool acquire_known_goal(void);

  /* goal acquisition states */

  /**
   * \brief Starting/reset state for FSM. Has no purpose other than that.
   */
  RCPPSW_HFSM_STATE_DECLARE_ND(acquire_goal_fsm, start);

  /**
   * \brief Main state for the FSM. Robots in this state vectoring to an a
   * derised object's location if any objects of the target type are known, and
   * explore for a desired object otherwise.
   */
  RCPPSW_HFSM_STATE_DECLARE_ND(acquire_goal_fsm, fsm_acquire_goal);

  /**
   * \brief Once a goal has been acquired, controller wait in this state until
   * reset by a higher level FSM.
   */
  RCPPSW_HFSM_STATE_DECLARE_ND(acquire_goal_fsm, finished);

  /**
   * \brief Simple state for exit in the main exploration state, used to reset
   * internal \ref vector_fsm and \ref explore_for_goal_fsm.
   * LED color for visualization purposes.
   */
  RCPPSW_HFSM_EXIT_DECLARE(acquire_goal_fsm, exit_fsm_acquire_goal);

  /**
   * \brief Defines the state map for the FSM.
   *
   * Note that the order of the states in the map MUST match the order of the
   * states in \enum states, or things will not work correctly.
   */
  RCPPSW_HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
    return &mc_state_map[index];
  }

  RCPPSW_HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ekST_MAX_STATES);

  /* clang-format off */
  rtypes::type_uuid              m_acq_id{rtypes::constants::kNoUUID};
  bool                           m_first_acq_step{false};
  hook_list                      m_hooks;
  vector_fsm                     m_vector_fsm;
  explore_for_goal_fsm           m_explore_fsm;
  /* clang-format on */
};

NS_END(fsm, spatial, cosm);

#endif /* INCLUDE_COSM_SPATIAL_FSM_ACQUIRE_GOAL_FSM_HPP_ */
