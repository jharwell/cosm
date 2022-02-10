/**
 * \file hal.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
 * Constant Definitions
 ******************************************************************************/
/*
 * \brief The configuration definition to compile for the footbot robot within
 * the ARGoS simulator.
 */
#define COSM_HAL_TARGET_ARGOS_FOOTBOT 1

/*
 * \brief The configuration definition to compile for the e-puck (an extended
 * version of the e-puck) robot within the ARGoS simulator. Originally from the
 * demiurge project: https://github.com/demiurge-project/argos3-epuck.
 *
 * I added a 3D physics model to it (by copying code from ARGoS).
 *
 * - The e-puck that comes with ARGoS has a 3D physics model, but doesn't have
 *   all the sensors I need.
 * - The pi-puck at https://github.com/allsey87/argos3-srocs also has a 3D
 *   physics model but not all the sensors I need.
 */
#define COSM_HAL_TARGET_ARGOS_EEPUCK3D 2

#define COSM_HAL_TARGET_ARGOS_PIPUCK 3

/**
 * \brief The configuration definition to compile for the real Turtlebot3 robot
 * for ROS.
 */
#define COSM_HAL_TARGET_ROS_TURTLEBOT3 4

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT) || (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D) || (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK)
#define COSM_HAL_TARGET_ARGOS_ROBOT
#endif
#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ROS_TURTLEBOT3)
#define COSM_HAL_TARGET_ROS_ROBOT
#endif

