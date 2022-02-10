/**
 * \file nest_zone_tracker.hpp
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
#include "cosm/spatial/metrics/nest_zone_metrics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class nest_zone_tracker
 * \ingroup spatial
 *
 * \brief Adapts \ref cfsm::state_tracker to tracking when a robot has
 * entered/exited the nest, and how long they stay in it for. Tracking is
 * performed regardless of why the robot enters the nest (exploration, homing,
 * etc.).
 */
class nest_zone_tracker final : public cfsm::state_tracker,
                                public csmetrics::nest_zone_metrics {
 public:
  explicit nest_zone_tracker(
      const csubsystem::sensing_subsystemQ3D* const sensing)
      : cfsm::state_tracker(sensing) {}


  /* Not move/copy constructable/assignable by default */
  nest_zone_tracker(const nest_zone_tracker&) = delete;
  nest_zone_tracker& operator=(const nest_zone_tracker&) = delete;
  nest_zone_tracker(nest_zone_tracker&&) = delete;
  nest_zone_tracker& operator=(nest_zone_tracker&&) = delete;

  bool in_nest(void) const override final {
    return cfsm::state_tracker::in_state();
  }
  bool entered_nest(void) const override final {
    return cfsm::state_tracker::entered_state();
  }
  bool exited_nest(void) const override final {
    return cfsm::state_tracker::exited_state();
  }
  rtypes::timestep nest_duration(void) const override final {
    return cfsm::state_tracker::state_duration();
  }
  rtypes::timestep nest_entry_time(void) const override final {
    return cfsm::state_tracker::state_entry_time();
  }
};

NS_END(spatial, cosm);

