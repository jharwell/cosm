/**
 * \file locking.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
namespace cosm::arena {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief Bitwise masking for expressing which \ref base_arena_map and \ref
 * caching_arena_map locks are already held when an operation is performed, so
 * the correct locks will be taken internally.
 */
enum class locking : uint {
  ekNONE_HELD = 1 << 0,
  ekBLOCKS_HELD = 1 << 1,
  ekCACHES_HELD = 1 << 2,
  ekGRID_HELD = 1 << 3,
  ekALL_HELD = ekNONE_HELD | ekBLOCKS_HELD | ekCACHES_HELD | ekGRID_HELD
};
} /* namespace cosm::arena */
