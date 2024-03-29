/**
 * \file dynamics_type.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::tv {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief The different types of swarm dynamics that the \ref tv_manager
 * manages.
 */
enum class dynamics_type { ekPOPULATION, ekENVIRONMENT };

} /* namespace cosm::tv */
