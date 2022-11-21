/**
 * \file interference_tracker.hpp
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
#include "cosm/spatial/metrics/interference_metrics.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::spatial {

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
      const csubsystem::sensing_subsystem* const sensing)
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
  boost::optional<rtypes::timestep> interference_duration(void) const override final;
  boost::optional<rmath::vector3z> interference_loc3D(void) const override final;
};

} /* namespace cosm::spatial */
