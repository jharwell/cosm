/**
 * \file locking.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/common/common.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, arena);

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
NS_END(arena, cosm);
