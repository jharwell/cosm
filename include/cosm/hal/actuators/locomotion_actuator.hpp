/**
 * \file locomotion_actuator.hpp
 *
 * SPDX-License-Identifier: MIT
  */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <utility>

#include "cosm/hal/actuators/base_actuator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::actuators {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class locomotion_actuator
 * \ingroup hal actuators
 *
 * \brief Base class for actuators that \a move an agent through space, as
 * opposed to other actuators of movement actuators which only move part of an
 * agent (e.g., those for repositioning a camera).
 */
class locomotion_actuator {
 public:
  locomotion_actuator(void) = default;
  virtual ~locomotion_actuator(void) = default;

  locomotion_actuator(const locomotion_actuator&) = default;
  locomotion_actuator& operator=(const locomotion_actuator&) = default;
  locomotion_actuator(locomotion_actuator&&) = default;
  locomotion_actuator& operator=(locomotion_actuator&&) = default;

  /**
   * \brief Stop all motion, with minimal rampdown.
   */
  virtual void stop(void) = 0;

  /**
   * \brief Get the maximum magnitude of a commanded velocity vector which the
   * actuator will accept.
   */
  virtual double max_velocity(void) const = 0;
};

} /* namespace cosm::hal::actuators */
