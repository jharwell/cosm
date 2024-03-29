/**
 * \file hal.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Constant Definitions
 ******************************************************************************/
/**
 * \brief The configuration definition to compile for the footbot robot within
 * the ARGoS simulator.
 *
 * Support for this robot platform is mature and robust.
 */
#define COSM_HAL_TARGET_ARGOS_FOOTBOT 1

/**
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
 *
 * Bindings for this robot are alpha-quality at best.
 */
#define COSM_HAL_TARGET_ARGOS_EEPUCK3D 2

/**
 * \brief The configuration definition to compiler for the pi-puck within the
 * ARGoS robot simulator.
 *
 * Bindings for this robot are alpha-quality at best.
 */
#define COSM_HAL_TARGET_ARGOS_PIPUCK 3

/**
 * \brief The configuration definition to compiler for the drone within the
 * ARGoS robot simulator.
 *
 * Bindings for this robot are alpha-quality at best.
 */
#define COSM_HAL_TARGET_ARGOS_DRONE 4

/**
 * \brief The configuration definition to compile for the real Turtlebot3 robot
 * for ROS with some additional sensors.
 *
 * Bindings for this robot are reasonably complete and mature, though some bugs
 * may exist.
 */
#define COSM_HAL_TARGET_ROS_ETURTLEBOT3 5

#if ((COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT) || \
     (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D) ||     \
     (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK) ||       \
     (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_DRONE))
#define COSM_HAL_TARGET_ARGOS_ROBOT
#endif

#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)

#define COSM_HAL_TARGET_HAS_ODOM_SENSOR

#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

#if ((COSM_HAL_TARGET == COSM_HAL_TARGET_ROS_ETURTLEBOT3))
#define COSM_HAL_TARGET_ROS_ROBOT
#endif
