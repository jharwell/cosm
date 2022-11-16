/**
 * \file time_estimate.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/ema.hpp"
#include "rcppsw/rcppsw.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief The type used to estimate time when performing calculations in task
 * allocation.
 */
using time_estimate = rmath::ema<int>;

NS_END(ta, cosm);
