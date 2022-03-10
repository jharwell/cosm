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

#include "cosm/kin2D/governed_diff_drive.hpp"
#include "cosm/hal/actuators/diagnostic_actuator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, subsystem);

#if COSM_HAL_TARGET == COSM_HAL_TARGET_ROS_ETURTLEBOT3
#define COSM_HAL_ROBOT_ACTUATOR_TYPES            \
  ckin2D::governed_diff_drive,                   \
    chactuators::diagnostic_actuator
#endif


/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class actuation_subsystem2D
 * \ingroup hal ros subsystem
 *
 * \brief The actuation subsystem for any ROS robot which operates in 2D.
 */
class actuation_subsystem2D :
    public chsubsystem::base_actuation_subsystem2D<COSM_HAL_ROBOT_ACTUATOR_TYPES> {
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
                             chactuators::diagnostic_actuator,
                             diagnostics);
  COSM_HAL_ACTUATOR_ACCESSOR(robot_actuator_types,
                             chactuators::diagnostic_actuator,
                             diagnostics,
                             const);
};

NS_END(subsystem, ros, hal, cosm);

