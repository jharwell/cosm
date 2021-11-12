/**
 * \file robot.hpp
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

#ifndef INCLUDE_COSM_HAL_ROBOT_HPP_
#define INCLUDE_COSM_HAL_ROBOT_HPP_

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
#endif /* COSM_HAL_TARGET */

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal);

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT)
using robot = ::argos::CFootBotEntity;
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
using robot = ::argos::CEPuckEntity;
#elif (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK)
using robot = ::argos::CPiPuckEntity;
#endif /* COSM_HAL_TARGET */

NS_END(hal, cosm);

#endif /* INCLUDE_COSM_HAL_ROBOT_HPP_ */
