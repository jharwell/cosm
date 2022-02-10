/**
 * \file coord_search_policy.hpp
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
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, foraging, block_dist);

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * \brief The policy to use when trying to find coordinates to distribute a
 * block to.
 */

enum coord_search_policy {
  /**
   * \brief Guess and check.
   */
  ekRANDOM,

  /**
   * \brief Compute a list of all the free cells in the distributable area and
   * then try each in turn to see if they will work.
   */
  ekFREE_CELL
};

NS_END(block_dist, foraging, cosm);

