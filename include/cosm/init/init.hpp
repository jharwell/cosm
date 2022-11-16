/**
 * \file init.hpp
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

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, init);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief Initialize the COSM library.
 *
 * Currently this just prints the version.
 */
void init(void) RCPPSW_LIB_INIT;

NS_END(init, cosm);
