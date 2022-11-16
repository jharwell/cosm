/**
 * \file swarm_manager.hpp
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
#include "cosm/pal/argos/swarm_manager_adaptor.hpp"
#elif defined(COSM_PAL_TARGET_ROS)
#include "cosm/pal/swarm_manager_adaptor.hpp"
#else
#error Selected platform has no swarm manager
#endif /* COSM_PAL_TARGET_ARGOS */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
#if defined(COSM_PAL_TARGET_ARGOS)
using swarm_manager = cpargos::swarm_manager_adaptor;
#elif defined(COSM_PAL_TARGET_ROS)
using swarm_manager = cpros::swarm_manager_adaptor;
#endif /* COSM_PAL_TARGET_ARGOS */

NS_END(pal, cosm);
