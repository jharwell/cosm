/**
 * \file robot_available_actuators.hpp
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
#include "cosm/cosm.hpp"
#include "cosm/kin2D/governed_diff_drive.hpp"
#include "cosm/hal/actuators/diagnostic_actuator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, ros, subsystem);

#if COSM_HAL_TARGET == COSM_HAL_TARGET_ROS_ETURTLEBOT3
#define COSM_HAL_ROBOT_AVAILABLE_ACTUATORS      \
  ckin2D::governed_diff_drive,                  \
    chactuators::diff_drive_actuator,           \
    chactuators::diagnostic_actuator
#endif


NS_END(subsystem, ros, hal, cosm);
