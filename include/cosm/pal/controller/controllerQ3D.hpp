/**
 * \file controllerQ3D.hpp
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

#ifndef INCLUDE_COSM_PAL_CONTROLLER_CONTROLLERQ3D_HPP_
#define INCLUDE_COSM_PAL_CONTROLLER_CONTROLLERQ3D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"
#include "cosm/pal/pal.hpp"

#if defined(COSM_PAL_TARGET_ARGOS)
#include "cosm/pal/argos/controller/adaptorQ3D.hpp"
#else
#error Selected platform does not support Q3D controllers
#endif /* COSM_PAL_TARGET_ARGOS */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal, controller);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
#if defined(COSM_PAL_TARGET_ARGOS)
using controllerQ3D = cpargos::controller::adaptorQ3D;
#elif defined(COSM_PAL_TARGET_ARGOS)
using controllerQ3D = cpros::controller::adaptorQ3D;
#endif /* COSM_PAL_TARGET_ARGOS */

NS_END(controller, pal, cosm);

#endif /* INCLUDE_COSM_PAL_CONTROLLER_CONTROLLERQ3D_HPP_ */
