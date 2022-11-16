/**
 * \file ds_variant.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <variant>
#include <memory>

#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, ds);
class bi_tdgraph;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief std::variant containing one of the possible data structures that the
 * \ref base_executive and its derived classes can operate on.
 */
using ds_variant = std::variant<std::unique_ptr<bi_tdgraph>>;

NS_END(ds, ta, cosm);
