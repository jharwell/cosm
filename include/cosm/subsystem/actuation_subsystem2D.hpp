/**
 * \file actuation_subsystem2D.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/subsystem/actuation_subsystem2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class actuation_subsystem2D
 * \ingroup subsystem
 *
 * \brief The  actuation subsystem for all actuators used by robot
 * controllers that operate in 2D.
 */
class actuation_subsystem2D : public chsubsystem::actuation_subsystem2D {
 public:
  explicit actuation_subsystem2D(actuator_map&& actuators)
      : chsubsystem::actuation_subsystem2D(std::move(actuators)) {}

  /* Not move/copy constructable/assignable by default */
  actuation_subsystem2D(const actuation_subsystem2D&) = delete;
  actuation_subsystem2D& operator=(const actuation_subsystem2D&) = delete;
  actuation_subsystem2D(actuation_subsystem2D&&) = delete;
  actuation_subsystem2D& operator=(actuation_subsystem2D&&) = delete;
};

NS_END(subsystem, cosm);
