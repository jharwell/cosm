/**
 * \file controller3D.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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
#include "cosm/pal/argos/controller/adaptor3D.hpp"
#else
#error Selected platform does not support 3D controllers
#endif /* COSM_PAL_TARGET_ARGOS */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::pal::controller {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
#if defined(COSM_PAL_TARGET_ARGOS)
using controller3D = cpargos::controller::adaptor3D;
#endif /* COSM_PAL_TARGET_ARGOS */

} /* namespace cosm::pal::controller */

