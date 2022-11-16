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

#include "rcppsw/common/licensing.hpp"

#include <iostream>

#include "cosm/version.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, init);

/*******************************************************************************
 * Free Functions
 ******************************************************************************/
void init(void) {
  std::cout << "Loaded COSM, " << kVERSION << ": ";
  std::cout << RCPPSW_LICENSE(MIT, COSM, 2022, John Harwell) << std::endl;
} /* init() */

NS_END(init, cosm);
