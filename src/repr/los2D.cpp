/**
 * \file los2D.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
 *****************************************************************************/
#include "cosm/repr/los2D.hpp"

#include "cosm/ds/cell2D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
const cds::cell2D& los2D::access(size_t i, size_t j) const {
  ER_ASSERT(i < xsize(),
            "Out of bounds X access: %zu >= %lu",
            i,
            xsize());
  ER_ASSERT(j < ysize(),
            "Out of bounds Y access: %zu >= %lu",
            j,
            ysize());
  return view()[i][j];
} /* access() */

NS_END(repr, cosm);
