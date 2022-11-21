/**
 * \file taskable_argument.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class taskable_argument
 * \ingroup ta
 *
 * \brief Base classthat any application specific data that should be passed to
 * tasks upon task start must derive from.
 */
class taskable_argument {
 public:
  taskable_argument(void) = default;
  virtual ~taskable_argument(void) = default;
};

} /* namespace cosm::ta */
