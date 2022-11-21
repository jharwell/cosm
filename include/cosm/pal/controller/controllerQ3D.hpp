/**
 * \file controllerQ3D.hpp
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
#include "cosm/pal/pal.hpp"

#if defined(COSM_PAL_TARGET_ARGOS)
#include "cosm/pal/argos/controller/adaptorQ3D.hpp"
#else
#error Selected platform does not support Q3D controllers
#endif /* COSM_PAL_TARGET_ARGOS */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::pal::controller {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
#if defined(COSM_PAL_TARGET_ARGOS)
using controllerQ3D = cpargos::controller::adaptorQ3D;
#endif /* COSM_PAL_TARGET_ARGOS */

} /* namespace cosm::pal::controller */

