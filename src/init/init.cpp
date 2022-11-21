/**
 * \file init.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/init/init.hpp"

#include "rcppsw/version/version.hpp"

#include <iostream>

#include "rcppsw/init/init.hpp"

#include "cosm/version/version.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::init {

/*******************************************************************************
 * Free Functions
 ******************************************************************************/
void init(void) {
  std::cout << "Loaded Core Swarm (COSM): ";
  std::cout << rversion::meta_info_to_str(&version::kVersion);
} /* init() */

} /* namespace cosm::init */
