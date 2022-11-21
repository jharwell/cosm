/**
 * \file block_variant.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <variant>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::repr {

class cube_block3D;
class ramp_block3D;
class base_block3D;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
using block3D_variantno = std::variant<cube_block3D*, ramp_block3D*>;
using block3D_variantro = std::variant<const cube_block3D*, const ramp_block3D*>;

/*******************************************************************************
 * Free Functions
 ******************************************************************************/
crepr::block3D_variantno make_variant(crepr::base_block3D* block);
crepr::block3D_variantro make_variant(const crepr::base_block3D* block);

} /* namespace cosm::repr */
