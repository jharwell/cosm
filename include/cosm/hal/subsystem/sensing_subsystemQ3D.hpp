/**
 * \file sensing_subsystemQ3D.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "cosm/hal/hal.hpp"
#include "cosm/cosm.hpp"

#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
#include "cosm/hal/argos/subsystem/sensing_subsystemQ3D.hpp"
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
#include "cosm/hal/ros/subsystem/sensing_subsystemQ3D.hpp"
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, subsystem);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
using sensing_subsystemQ3D = chargos::subsystem::sensing_subsystemQ3D;
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
using sensing_subsystemQ3D = chros::subsystem::sensing_subsystemQ3D;
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

NS_END(subsystem, hal, cosm);
