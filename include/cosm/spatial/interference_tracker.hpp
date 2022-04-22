/**
 * \file interference_tracker.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/fsm/state_tracker.hpp"
#include "cosm/spatial/metrics/interference_metrics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class interference_tracker
 * \ingroup spatial
 *
 * \brief Adapts \ref cfsm::state_tracker to tracking when a robot
 * experiences interference.
 */
class interference_tracker final : public cfsm::state_tracker,
                                   public csmetrics::interference_metrics {
 public:
  explicit interference_tracker(
      const csubsystem::sensing_subsystemQ3D* const sensing)
      : state_tracker(sensing) {}

  /* Not move/copy constructable/assignable by default */
  interference_tracker(const interference_tracker&) = delete;
  interference_tracker& operator=(const interference_tracker&) = delete;
  interference_tracker(interference_tracker&&) = delete;
  interference_tracker& operator=(interference_tracker&&) = delete;

  /* interference metrics */
  bool exp_interference(void) const override final {
    return state_tracker::in_state();
  }
  bool entered_interference(void) const override final {
    return state_tracker::entered_state();
  }
  bool exited_interference(void) const override final {
    return state_tracker::exited_state();
  }
  rtypes::timestep interference_duration(void) const override final {
    return state_tracker::state_duration();
  }
  rmath::vector3z interference_loc3D(void) const override final {
    return state_tracker::state_loc3D();
  }
};

NS_END(spatial, cosm);
