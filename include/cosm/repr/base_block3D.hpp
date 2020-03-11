/**
 * \file base_block3D.hpp
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

#ifndef INCLUDE_COSM_REPR_BASE_BLOCK3D_HPP_
#define INCLUDE_COSM_REPR_BASE_BLOCK3D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/repr/base_block.hpp"
#include "cosm/repr/unicell_movable_entity3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
using base_block3D = base_block<unicell_movable_entity3D>;

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_BASE_BLOCK3D_HPP_ */
