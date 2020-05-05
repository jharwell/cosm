/**
 * \file vector_fsm.hpp
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

#ifndef INCLUDE_COSM_SPATIAL_FSM_VECTOR_FSM_HPP_
#define INCLUDE_COSM_SPATIAL_FSM_VECTOR_FSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"

#include "cosm/cosm.hpp"
#include "cosm/spatial/fsm/util_hfsm.hpp"
#include "cosm/ta/taskable.hpp"
#include "cosm/spatial/fsm/point_argument.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class vector_fsm
 * \ingroup spatial fsm
 *
 * \brief An FSM used to send a robot to a particular ABSOLUTE location in the
 * arena.
 *
 * Vectoring is controlled by two PID loops: one for angle between robot heading
 * and the heading to the goal, and one for distance of robot to the goal.
 *
 * Arrival tolerance can be specified differently depending on what the goal is.
 */
class vector_fsm final : public csfsm::util_hfsm,
                         public rer::client<vector_fsm>,
                         public cta::taskable {
 public:
  vector_fsm(subsystem::saa_subsystemQ3D* saa, rmath::rng* rng);

  vector_fsm& operator=(const vector_fsm&) = delete;
  vector_fsm(const vector_fsm&) = delete;

  /* taskable overrides */
  void task_reset(void) override { init(); }
  bool task_running(void) const override {
    return current_state() != ekST_START && current_state() != ekST_ARRIVED;
  }
  void task_execute(void) override;
  void task_start(ta::taskable_argument* c_arg) override;
  bool task_finished(void) const override {
    return current_state() == ekST_ARRIVED;
  }

  const rmath::vector2d& target(void) const { return m_goal.point(); }

  /* HFSM overrides */
  /**
   * \brief Initialize/re-initialize the FSM. After arriving at a
   * goal, this function must be called before vectoring to a new goal will
   * work.
   */
  void init(void) override;

  /* collision metrics */
  bool in_collision_avoidance(void) const override RCSW_PURE;
  bool entered_collision_avoidance(void) const override RCSW_PURE;
  bool exited_collision_avoidance(void) const override RCSW_PURE;
  rmath::vector2z avoidance_loc2D(void) const override;
  rmath::vector3z avoidance_loc3D(void) const override;

 private:
  enum state {
    ekST_START,
    /**
     * Vectoring toward the target.
     */
    ekST_VECTOR,

    /**
     * Avoiding an obstacle nearby to the robot's current location.
     */
    ekST_COLLISION_AVOIDANCE,

    /**
     * Recovering from frequent collision avoidance by driving AWAY from the
     * site of the most recent collision in a random direction for a set number
     * of timesteps. This is intended to help prevent robot's from wasting lots
     * of time butting heads when they are traveling in opposite/spatially
     * conflicting directions.
     */
    ekST_COLLISION_RECOVERY,

    /**
     * We have arrived at the specified location within tolerance.
     */
    ekST_ARRIVED,
    ekST_MAX_STATES
  };

  struct fsm_state {
    uint m_collision_rec_count{0};
  };

  /**
   * \brief The # of timesteps according to collision recovery. This is mainly
   * to ensure that you do not repeatedly get 2 controller butting heads as they
   * try to travel to opposite goals.
   */
  static constexpr const uint kCOLLISION_RECOVERY_TIME = 10;

  /**
   * \brief Calculates the relative vector from the robot to the current goal.
   *
   * \param goal The current goal.
   *
   * \return The vector, specified with the tail at the robot and the head
   * pointing towards the goal.
   */
  rmath::vector2d calc_vector_to_goal(const rmath::vector2d& goal) RCSW_PURE;

  /* vector states */
  HFSM_STATE_DECLARE_ND(vector_fsm, start);
  HFSM_STATE_DECLARE_ND(vector_fsm, vector);
  HFSM_STATE_DECLARE_ND(vector_fsm, collision_avoidance);
  HFSM_STATE_DECLARE_ND(vector_fsm, collision_recovery);
  HFSM_STATE_DECLARE(vector_fsm, arrived, point_argument);

  HFSM_ENTRY_DECLARE_ND(vector_fsm, entry_vector);
  HFSM_ENTRY_DECLARE_ND(vector_fsm, entry_collision_avoidance);
  HFSM_ENTRY_DECLARE_ND(vector_fsm, entry_collision_recovery);

  HFSM_EXIT_DECLARE(vector_fsm, exit_collision_avoidance);

  HFSM_DEFINE_STATE_MAP_ACCESSOR(state_map_ex, index) override {
    return &mc_state_map[index];
  }

  HFSM_DECLARE_STATE_MAP(state_map_ex, mc_state_map, ekST_MAX_STATES);

  /* clang-format off */
  fsm_state      m_state{};
  point_argument m_goal{};
  /* clang-format on */
};

NS_END(fsm, spatial, cosm);

#endif /* INCLUDE_COSM_SPATIAL_FSM_VECTOR_FSM_HPP_ */
