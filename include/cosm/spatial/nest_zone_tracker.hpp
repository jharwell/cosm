/**
 * \file nest_zone_tracker.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
  explicit nest_zone_tracker(const csubsystem::sensing_subsystemQ3D* const sensing)
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
