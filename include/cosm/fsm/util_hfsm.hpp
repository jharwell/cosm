/**
 * @file util_hfsm.hpp
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

#ifndef INCLUDE_COSM_FSM_UTIL_HFSM_HPP_
#define INCLUDE_COSM_FSM_UTIL_HFSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <random>
#include <string>

#include "rcppsw/math/vector2.hpp"
#include "rcppsw/patterns/fsm/hfsm.hpp"

#include "cosm/cosm.hpp"
#include "cosm/fsm/collision_tracker.hpp"
#include "cosm/fsm/metrics/collision_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm);

namespace subsystem {
class saa_subsystem2D;
class sensing_subsystem2D;
class actuation_subsystem2D;
} // namespace subsystem

NS_START(fsm);

struct new_direction_data;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class util_hfsm
 * @ingroup cosm fsm
 *
 * @brief A collection of base states/common functionality that FSMs can use if
 * they choose.
 *
 * This class cannot be instantiated on its own, as does not define an FSM
 * per-se.
 */
class util_hfsm : public rpfsm::hfsm,
                  public rer::client<util_hfsm>,
                  public metrics::collision_metrics {
 public:
  util_hfsm(subsystem::saa_subsystem2D* saa, uint8_t max_states);

  ~util_hfsm(void) override = default;

  util_hfsm(const util_hfsm& fsm) = delete;
  util_hfsm& operator=(const util_hfsm& fsm) = delete;

  virtual subsystem::sensing_subsystem2D* sensing(void);
  virtual const subsystem::sensing_subsystem2D* sensing(void) const;

  virtual subsystem::actuation_subsystem2D* actuation(void);
  virtual const subsystem::actuation_subsystem2D* actuation(void) const;

  const collision_tracker* ca_tracker(void) const { return &m_tracker; }

 protected:
  /**
   * @brief Calculate a random angle in [0, pi] for the purposes of direction
   * change.
   */
  rmath::radians random_angle(void);

  const subsystem::saa_subsystem2D* saa(void) const { return m_saa; }
  subsystem::saa_subsystem2D* saa(void) { return m_saa; }
  collision_tracker* ca_tracker(void) { return &m_tracker; }

  /**
   * @brief Robots entering this state will return to the nest.
   *
   * This state MUST have a parent state defined that is not
   * \ref rpatterns::fsm::hfsm::top_state().
   */
  HFSM_STATE_DECLARE(util_hfsm, transport_to_nest, rpfsm::event_data);

  /**
   * @brief Robots entering this state will leave the nest (they are assumed to
   * already be in the nest when this state is entered).
   *
   * This state MUST have a parent state defined that is not \ref
   * hfsm::top_state().
   */
  HFSM_STATE_DECLARE(util_hfsm, leaving_nest, rpfsm::event_data);

  /**
   * @brief Robots entering this state will randomly change their exploration
   * direction to the specified direction. All signals are ignored in this
   * state. Once the direction change has been accomplished, the robot will
   * transition back to its previous state.
   */
  HFSM_STATE_DECLARE(util_hfsm, new_direction, rpfsm::event_data);

  /**
   * @brief Entry state for returning to nest.
   *
   * Used to:
   *
   * - Set LED colors for visualization purposes.
   * - Enable light sensor (disabled otherwise for computational
   * - efficiency).
   */
  HFSM_ENTRY_DECLARE_ND(util_hfsm, entry_transport_to_nest);

  /**
   * @brief A simple entry state for leaving nest, used to set LED colors for
   * visualization purposes.
   */
  HFSM_ENTRY_DECLARE_ND(util_hfsm, entry_leaving_nest);

  /**
   * @brief Simple state for entry into the new direction state, used to change
   * LED color for visualization purposes.
   */
  HFSM_ENTRY_DECLARE_ND(util_hfsm, entry_new_direction);

  /**
   * @brief Simple state for entry into the "wait for signal" state, used to
   * change LED color for visualization purposes.
   */
  HFSM_ENTRY_DECLARE_ND(util_hfsm, entry_wait_for_signal);

  /**
   * @brief Exit state for returning to nest (i.e. when the robot arrives in the
   * nest).
   *
   * Used to:
   *
   * - Set LED colors for visualization purposes.
   * - Disable light sensor (disabled unless a robot is activiely returning to
   * - the nest for computational efficiency).
   */
  HFSM_EXIT_DECLARE(util_hfsm, exit_transport_to_nest);

 private:
  /**
   * @brief When changing direction, a robot is spinning at such a speed that it
   * may overshoot its desired new direction, but as long as it does not
   * overshoot by more than this tolerance, the direction change will still be
   * considered to have occurred successfully.
   */
  static constexpr double kDIR_CHANGE_TOL = 0.25;

  /**
   * @brief When changing direction, it may not be enough to have an arrival
   * tolerance for the new heading; it is possible that given the new direction,
   * the robot's initial heading, and the spinning speed, and it is impossible
   * for the robot to arrive within tolerance of the desired new direction. So,
   * I also define a max number of steps that the robot will spin as a secondary
   * safeguard.
   *
   * Not doing this leads to controller that will spin more or less indefinitely
   * when changing direction on occasion.
   */
  static constexpr uint kDIR_CHANGE_MAX_STEPS = 10;

  /**
   * @brief When entering the nest, you want to continue to wander a bit before
   * signaling upper FSMs that you are in the nest, so that there is (slightly)
   * less congestion by the edge. This is a stopgap solution; a more elegant fix
   * may be forthcoming in the future if warranted.
   */
  static constexpr uint kNEST_COUNT_MAX_STEPS = 25;

  /* clang-format off */
  uint                              m_nest_count{0};
  uint                              m_new_dir_count{0};
  rmath::radians                    m_new_dir{};
  std::default_random_engine        m_rng;
  subsystem::saa_subsystem2D* const m_saa;
  collision_tracker                 m_tracker;
  /* clang-format on */

 public:
  /* collision metrics */
  RCPPSW_DECLDEF_OVERRIDE_WRAP(in_collision_avoidance, m_tracker, const)
  RCPPSW_DECLDEF_OVERRIDE_WRAP(entered_collision_avoidance, m_tracker, const)
  RCPPSW_DECLDEF_OVERRIDE_WRAP(exited_collision_avoidance, m_tracker, const)
  RCPPSW_DECLDEF_OVERRIDE_WRAP(collision_avoidance_duration, m_tracker, const)
  RCPPSW_DECLDEF_OVERRIDE_WRAP(avoidance_loc, m_tracker, const)
};

NS_END(fsm, cosm);

#endif /* INCLUDE_COSM_FSM_UTIL_HFSM_HPP_ */
