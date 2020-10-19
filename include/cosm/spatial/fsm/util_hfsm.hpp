/**
 * \file util_hfsm.hpp
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

#ifndef INCLUDE_COSM_SPATIAL_FSM_UTIL_HFSM_HPP_
#define INCLUDE_COSM_SPATIAL_FSM_UTIL_HFSM_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>
#include <boost/optional.hpp>

#include "rcppsw/math/rng.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/patterns/fsm/hfsm.hpp"
#include "rcppsw/types/spatial_dist.hpp"

#include "cosm/cosm.hpp"
#include "cosm/spatial/interference_tracker.hpp"
#include "cosm/spatial/metrics/interference_metrics.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem {
class saa_subsystemQ3D;
class sensing_subsystemQ3D;
class actuation_subsystem2D;
} /* namespace cosm::subsystem */

NS_START(cosm, spatial, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class util_hfsm
 * \ingroup spatial fsm
 *
 * \brief A collection of base states/common functionality that robot FSMs which
 * operate in strict 2D or quasi-3D can use if they choose (NOT full 3D robots).
 *
 * This class cannot be instantiated on its own, as does not define an FSM
 * per-se.
 */
class util_hfsm : public rpfsm::hfsm,
                  public rer::client<util_hfsm>,
                  public metrics::interference_metrics {
 public:
  util_hfsm(csubsystem::saa_subsystemQ3D* saa,
            rmath::rng* rng,
            uint8_t max_states);

  ~util_hfsm(void) override = default;

  util_hfsm(const util_hfsm&) = delete;
  util_hfsm& operator=(const util_hfsm&) = delete;

  csubsystem::sensing_subsystemQ3D* sensing(void);
  const csubsystem::sensing_subsystemQ3D* sensing(void) const;

  typename csubsystem::actuation_subsystem2D* actuation(void);
  typename csubsystem::actuation_subsystem2D* actuation(void) const;

  /**
   * \brief Handle to internal interference tracker; provided as a common
   * utility to derived classes that do not utilize/wrap a \ref
   * spatial::expstrat::base_expstrat which has its own tracker.
   */
  const interference_tracker* inta_tracker(void) const { return &m_tracker; }

 protected:
  struct nest_transport_data : public rpfsm::event_data {
    explicit nest_transport_data(const rmath::vector2d& in)
        : nest_loc(in) {}
    rmath::vector2d nest_loc;
  };

  /**
   * \brief Calculate a random angle in [0, pi] for the purposes of direction
   * change.
   */
  rmath::radians random_angle(void);
  rmath::rng* rng(void) { return m_rng; }
  const rmath::rng* rng(void) const { return m_rng; }

  const csubsystem::saa_subsystemQ3D* saa(void) const { return m_saa; }
  csubsystem::saa_subsystemQ3D* saa(void) { return m_saa; }
  interference_tracker* inta_tracker(void) { return &m_tracker; }

  /**
   * \brief Robots entering this state will return to the nest.
   *
   * Once they enter the nest, robots:
   *
   * 1. Wander for a set amount of time in order to reduce congestion in the
   *    nest.
   * 2. Signal to the parent FSM that they have entered the nest and stop
   *    moving.
   *
   * \todo The wander behavior is somewhat foraging specific, and so this state
   *       may be of limited utility in other applications.
   *
   * This state MUST have a parent state defined that is not
   * \ref rcppsw::patterns::fsm::hfsm::top_state().
   */
  HFSM_STATE_DECLARE(util_hfsm, transport_to_nest, nest_transport_data);

  /**
   * \brief Robots entering this state will leave the nest (they are assumed to
   * already be in the nest when this state is entered).
   *
   * This state MUST have a parent state defined that is not \ref
   * hfsm::top_state().
   */
  HFSM_STATE_DECLARE(util_hfsm, leaving_nest, rpfsm::event_data);

  /**
   * \brief Entry state for returning to nest.
   *
   * Used to:
   *
   * - Set LED colors for visualization purposes.
   * - Enable light sensor (disabled otherwise for computational
   *   efficiency).
   */
  HFSM_ENTRY_DECLARE_ND(util_hfsm, entry_transport_to_nest);

  /**
   * \brief A simple entry state for leaving nest, used to set LED colors for
   * visualization purposes.
   */
  HFSM_ENTRY_DECLARE_ND(util_hfsm, entry_leaving_nest);

  /**
   * \brief Simple state for entry into the "wait for signal" state, used to
   * change LED color for visualization purposes.
   */
  HFSM_ENTRY_DECLARE_ND(util_hfsm, entry_wait_for_signal);

  /**
   * \brief Exit state for returning to nest (i.e. when the robot arrives in the
   * nest).
   *
   * Used to:
   *
   * - Set LED colors for visualization purposes.
   * - Disable light sensor (disabled unless a robot is activiely returning to
   *   the nest for computational efficiency).
   */
  HFSM_EXIT_DECLARE(util_hfsm, exit_transport_to_nest);

 private:
  /**
   * \brief When entering the nest, you want to continue to wander a bit before
   * signaling upper FSMs that you are in the nest, so that there is (slightly)
   * less congestion by the edge. This is a stopgap solution; a more elegant fix
   * may be forthcoming in the future if warranted.
   */
  static constexpr const size_t kNEST_COUNT_MAX_STEPS = 25;

  /* clang-format off */
  boost::optional<rtypes::spatial_dist> m_nest_thresh{0.0};
  csubsystem::saa_subsystemQ3D* const   m_saa;
  interference_tracker                  m_tracker;
  rmath::rng*                           m_rng;
  /* clang-format on */

 public:
  /* interference metrics */
  RCPPSW_DECLDEF_WRAP_OVERRIDE(exp_interference, m_tracker, const)
  RCPPSW_DECLDEF_WRAP_OVERRIDE(entered_interference, m_tracker, const)
  RCPPSW_DECLDEF_WRAP_OVERRIDE(exited_interference, m_tracker, const)
  RCPPSW_DECLDEF_WRAP_OVERRIDE(interference_duration, m_tracker, const)
  RCPPSW_DECLDEF_WRAP_OVERRIDE(interference_loc3D, m_tracker, const)
};

NS_END(fsm, spatial, cosm);

#endif /* INCLUDE_COSM_SPATIAL_FSM_UTIL_HFSM_HPP_ */
