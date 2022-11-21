/**
 * \file actuation_subsystem.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/subsystem/actuation_subsystem.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class actuation_subsystem
 * \ingroup subsystem
 *
 * \brief The  actuation subsystem for all actuators used by robot
 * controllers that operate in .
 */
class actuation_subsystem : public chsubsystem::actuation_subsystem {
 public:
  explicit actuation_subsystem(actuator_map&& actuators)
      : chsubsystem::actuation_subsystem(std::move(actuators)) {}

  /* Not move/copy constructable/assignable by default */
  actuation_subsystem(const actuation_subsystem&) = delete;
  actuation_subsystem& operator=(const actuation_subsystem&) = delete;
  actuation_subsystem(actuation_subsystem&&) = delete;
  actuation_subsystem& operator=(actuation_subsystem&&) = delete;
};

} /* namespace cosm::subsystem */
