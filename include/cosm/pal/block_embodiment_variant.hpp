/**
 * \file block_embodiment_variant.hpp
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

#ifndef INCLUDE_COSM_PAL_BLOCK_EMBODIMENT_VARIANT_HPP_
#define INCLUDE_COSM_PAL_BLOCK_EMBODIMENT_VARIANT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant.hpp>
#include <memory>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal);

struct ramp_block_embodiment;
struct cube_block_embodiment;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
using block_embodiment_variant = boost::variant<std::unique_ptr<ramp_block_embodiment>,
                                                std::unique_ptr<cube_block_embodiment>>;

NS_END(pal, cosm);

#endif /* INCLUDE_COSM_PAL_BLOCK_EMBODIMENT_VARIANT_HPP_ */
