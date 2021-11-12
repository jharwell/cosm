/**
 * \file embodied_block_variant.hpp
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

#ifndef INCLUDE_COSM_PAL_ARGOS_EMBODIED_BLOCK_VARIANT_HPP_
#define INCLUDE_COSM_PAL_ARGOS_EMBODIED_BLOCK_VARIANT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant.hpp>
#include <memory>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::repr {
class base_block3D;
} /* namespace cosm::repr */

NS_START(cosm, pal, argos);

class embodied_ramp_block;
class embodied_cube_block;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
using embodied_block_varianto = boost::variant<std::unique_ptr<embodied_ramp_block>,
                                              std::unique_ptr<embodied_cube_block>>;

using embodied_block_variantno = boost::variant<embodied_ramp_block*,
                                                embodied_cube_block*>;

NS_END(argos, pal, cosm);

#endif /* INCLUDE_COSM_PAL_ARGOS_EMBODIED_BLOCK_VARIANT_HPP_ */
