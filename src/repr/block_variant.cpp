/**
 * \file block_variant.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/repr/block_variant.hpp"

#include "cosm/repr/cube_block3D.hpp"
#include "cosm/repr/ramp_block3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Free Functions
 ******************************************************************************/
crepr::block3D_variantno make_variant(crepr::base_block3D* block) {
  if (crepr::block_type::ekCUBE == block->md()->type()) {
    return { static_cast<crepr::cube_block3D*>(block) };
  } else if (crepr::block_type::ekRAMP == block->md()->type()) {
    return { static_cast<crepr::ramp_block3D*>(block) };
  } else {
    return {};
  }
} /* make_variant() */

crepr::block3D_variantro make_variant(const crepr::base_block3D* block) {
  if (crepr::block_type::ekCUBE == block->md()->type()) {
    return { static_cast<const crepr::cube_block3D*>(block) };
  } else if (crepr::block_type::ekRAMP == block->md()->type()) {
    return { static_cast<const crepr::ramp_block3D*>(block) };
  } else {
    return {};
  }
} /* make_variant() */

NS_END(repr, cosm);
