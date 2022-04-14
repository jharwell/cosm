/**
 * \file actuation_subsystem2D.hpp
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
