/**
 * \file controller2D.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"
#include "cosm/hal/hal.hpp"
#include "cosm/pal/pal.hpp"

#if defined(COSM_PAL_TARGET_ARGOS)
#include "cosm/pal/argos/controller/adaptor2D.hpp"
#elif defined(COSM_PAL_TARGET_ROS)
#include "cosm/pal/ros/controller/adaptor2D.hpp"
#else
#error Selected platform does not support 2D controllers
#endif /* COSM_PAL_TARGET_ARGOS */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::pal::controller {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
#if defined(COSM_PAL_TARGET_ARGOS)
using controller2D = cpargos::controller::adaptor2D;
#elif defined(COSM_PAL_TARGET_ROS)
using controller2D = cpros::controller::adaptor2D;
#endif /* COSM_PAL_TARGET_ARGOS */

} /* namespace cosm::pal::controller */

