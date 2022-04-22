/**
 * \file actuation_subsystem2D.hpp
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
#include "cosm/hal/subsystem/base_actuation_subsystem2D.hpp"

#include "cosm/hal/argos/subsystem/robot_available_actuators.hpp"

#include "cosm/hal/actuators/diagnostic_actuator.hpp"
#include "cosm/hal/argos/actuators/wifi_actuator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, argos, subsystem);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class actuation_subsystem2D
 * \ingroup hal argos subsystem
 *
 * \brief The actuation subsystem for any ARGoS robot which operates in 2D.
 */
class actuation_subsystem2D :
    public chsubsystem::base_actuation_subsystem2D<COSM_HAL_ROBOT_AVAILABLE_ACTUATORS> {
 public:
  explicit actuation_subsystem2D(actuator_map&& actuators)
      : base_actuation_subsystem2D(std::move(actuators)) {}

  COSM_HAL_ACTUATOR_ACCESSOR(robot_actuator_types,
                             ckin2D::governed_diff_drive,
                             governed_diff_drive);
  COSM_HAL_ACTUATOR_ACCESSOR(robot_actuator_types,
                             ckin2D::governed_diff_drive,
                             governed_diff_drive,
                             const);

  COSM_HAL_ACTUATOR_ACCESSOR(robot_actuator_types,
                             chactuators::diff_drive_actuator,
                             diff_drive_raw);
  COSM_HAL_ACTUATOR_ACCESSOR(robot_actuator_types,
                             chactuators::diff_drive_actuator,
                             diff_drive_raw,
                             const);

  COSM_HAL_ACTUATOR_ACCESSOR(robot_actuator_types,
                             chal::actuators::diagnostic_actuator,
                             diagnostics);
  COSM_HAL_ACTUATOR_ACCESSOR(robot_actuator_types,
                             chal::actuators::diagnostic_actuator,
                             diagnostics,
                             const);

  COSM_HAL_ACTUATOR_ACCESSOR(robot_actuator_types,
                             chargos::actuators::wifi_actuator,
                             rab);
  COSM_HAL_ACTUATOR_ACCESSOR(robot_actuator_types,
                             chargos::actuators::wifi_actuator,
                             rab,
                             const);
};

NS_END(subsystem, argos, hal, cosm);
