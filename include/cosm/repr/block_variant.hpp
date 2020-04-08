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
#include <boost/variant.hpp>

#include "cosm/cosm.hpp"
#include "cosm/repr/base_block2D.hpp"
#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

class cube_block2D;
class ramp_block2D;
class cube_block3D;
class ramp_block3D;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
using block2D_variant = boost::variant<cube_block2D*,
                                       ramp_block2D*>;
using block3D_variant = boost::variant<cube_block3D*,
                                       ramp_block3D*>;
using block_variant = boost::variant<cube_block2D*,
                                     ramp_block2D*,
                                     cube_block3D*,
                                     ramp_block3D*>;
using base_block_variant = boost::variant<boost::blank,
                                          base_block2D*,
                                          base_block3D*>;
NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_BLOCK_VARIANT_HPP_ */
