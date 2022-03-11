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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <string>

#include "rcppsw/math/rng.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/patterns/fsm/hfsm.hpp"
#include "rcppsw/types/spatial_dist.hpp"

#include "cosm/cosm.hpp"
#include "cosm/spatial/interference_tracker.hpp"
#include "cosm/spatial/metrics/interference_metrics.hpp"
#include "cosm/spatial/strategy/base_strategy.hpp"
#include "cosm/spatial/nest_zone_tracker.hpp"
#include "cosm/spatial/metrics/nest_zone_metrics.hpp"
#include "cosm/subsystem/subsystem_fwd.hpp"
#include "cosm/spatial/fsm/fsm_params.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
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
                                public metrics::interference_metrics,
                                public metrics::nest_zone_metrics {
 public:
  util_hfsm(const fsm_params* params,
            rmath::rng* rng,
            uint8_t max_states);

  ~util_hfsm(void) override = default;

  util_hfsm(const util_hfsm&) = delete;
  util_hfsm& operator=(const util_hfsm&) = delete;

  void init(void) override;
  csubsystem::sensing_subsystemQ3D* sensing(void) RCPPSW_PURE;
  const csubsystem::sensing_subsystemQ3D* sensing(void) const RCPPSW_PURE;

  typename csubsystem::actuation_subsystem2D* actuation(void) RCPPSW_PURE;
  typename csubsystem::actuation_subsystem2D* actuation(void) const RCPPSW_PURE;

  /**
   * \brief Handle to internal interference tracker; provided as a common
   * utility to derived classes that do not utilize/wrap a \ref
   * spatial::strategy::base_strategy which has its own tracker.
   */
  const interference_tracker* inta_tracker(void) const { return m_inta_tracker; }

  /**
   * \brief Handle to internal nest zone tracker; provided as a common utility
   * to derived classes that do not utilize/wrap a \ref
   * spatial::strategy::base_strategy which has its own tracker.
   */
  const nest_zone_tracker* nz_tracker(void) const { return m_nz_tracker; }

 protected:
  /**
   * \brief Calculate a random angle in [0, pi] for the purposes of direction
   * change.
   */
  rmath::radians random_angle(void);
  rmath::rng* rng(void) { return m_rng; }
  const rmath::rng* rng(void) const { return m_rng; }

  const csubsystem::saa_subsystemQ3D* saa(void) const { return m_saa; }
  csubsystem::saa_subsystemQ3D* saa(void) { return m_saa; }
  interference_tracker* inta_tracker(void) { return m_inta_tracker; }
  nest_zone_tracker* nz_tracker(void) { return m_nz_tracker; }

  /**
   * \brief Simple state for entry into the "wait for signal" state, used to
   * change LED color for visualization purposes.
   */
  RCPPSW_HFSM_ENTRY_DECLARE_ND(util_hfsm, entry_wait_for_signal);

 private:
  /* clang-format off */
  csubsystem::saa_subsystemQ3D* const m_saa;
  interference_tracker*               m_inta_tracker;
  nest_zone_tracker*                  m_nz_tracker;

  rmath::rng*                         m_rng;
  /* clang-format on */

 public:
  /* interference metrics */
  RCPPSW_WRAP_DECLDEF_OVERRIDE(exp_interference, *m_inta_tracker, const)
  RCPPSW_WRAP_DECLDEF_OVERRIDE(entered_interference, *m_inta_tracker, const)
  RCPPSW_WRAP_DECLDEF_OVERRIDE(exited_interference, *m_inta_tracker, const)
  RCPPSW_WRAP_DECLDEF_OVERRIDE(interference_duration, *m_inta_tracker, const)
  RCPPSW_WRAP_DECLDEF_OVERRIDE(interference_loc3D, *m_inta_tracker, const)

  /* nest_zone metrics */
  RCPPSW_WRAP_DECLDEF_OVERRIDE(in_nest, *m_nz_tracker, const);
  RCPPSW_WRAP_DECLDEF_OVERRIDE(entered_nest, *m_nz_tracker, const);
  RCPPSW_WRAP_DECLDEF_OVERRIDE(exited_nest, *m_nz_tracker, const);
  RCPPSW_WRAP_DECLDEF_OVERRIDE(nest_duration, *m_nz_tracker, const);
  RCPPSW_WRAP_DECLDEF_OVERRIDE(nest_entry_time, *m_nz_tracker, const);
};

NS_END(fsm, spatial, cosm);

