/**
 * \file block_variant.hpp
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

#ifndef INCLUDE_COSM_REPR_BLOCK_VARIANT_HPP_
#define INCLUDE_COSM_REPR_BLOCK_VARIANT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <variant>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

class cube_block3D;
class ramp_block3D;
class base_block3D;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
using block3D_variantno = std::variant<cube_block3D*, ramp_block3D*>;
using block3D_variantro = std::variant<const cube_block3D*,
                                         const ramp_block3D*>;

/*******************************************************************************
 * Free Functions
 ******************************************************************************/
crepr::block3D_variantno make_variant(crepr::base_block3D* block);
crepr::block3D_variantro make_variant(const crepr::base_block3D* block);

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_BLOCK_VARIANT_HPP_ */
