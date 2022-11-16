/**
 * \file block_variant.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
