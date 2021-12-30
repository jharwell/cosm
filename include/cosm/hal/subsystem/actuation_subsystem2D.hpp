/**
 * \file actuation_subsystem2D.hpp
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

#ifndef INCLUDE_COSM_HAL_SUBSYSTEM_ACTUATION_SUBSYSTEM2D_HPP_
#define INCLUDE_COSM_HAL_SUBSYSTEM_ACTUATION_SUBSYSTEM2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/hal.hpp"
#include "cosm/cosm.hpp"

#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
#include "cosm/hal/argos/subsystem/actuation_subsystem2D.hpp"
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
#include "cosm/hal/ros/subsystem/actuation_subsystem2D.hpp"
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, subsystem);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
using actuation_subsystem2D = chargos::subsystem::actuation_subsystem2D;
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
using actuation_subsystem2D = chros::subsystem::actuation_subsystem2D;
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

NS_END(subsystem, hal, cosm);

#endif /* INCLUDE_COSM_HAL_SUBSYSTEM_ACTUATION_SUBSYSTEM2D_HPP_ */
