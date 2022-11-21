/**
 * \file robot.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/hal.hpp"

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT)
#include <argos3/plugins/robots/foot-bot/simulator/footbot_entity.h>
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK)
#include <argos3/plugins/robots/pi-puck/simulator/pipuck_entity.h>
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_DRONE)
#include <argos3/plugins/robots/drone/simulator/drone_entity.h>
#endif /* COSM_HAL_TARGET */

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal {

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT)
using robot = ::argos::CFootBotEntity;
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
using robot = ::argos::CEPuckEntity;
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK)
using robot = ::argos::CPiPuckEntity;
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_DRONE)
using robot = ::argos::CDroneEntity;
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ROS_ETURTLEBOT3)
struct robot {};
#endif /* COSM_HAL_TARGET */

} /* namespace cosm::hal */
