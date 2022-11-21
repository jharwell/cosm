/**
 * \file swarm_visualizer.hpp
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
#include "cosm/pal/argos/swarm_visualizer_adaptor.hpp"
#else
#error Selected platform has no swarm visualizer
#endif /* COSM_PAL_TARGET_ARGOS */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::pal {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
#if defined(COSM_PAL_TARGET_ARGOS)
using swarm_visualizer = cpargos::swarm_visualizer_adaptor;
#endif /* COSM_PAL_TARGET_ARGOS */

} /* namespace cosm::pal */
