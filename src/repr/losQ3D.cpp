/**
 * \file losQ3D.cpp
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

/*******************************************************************************
 * Includes
 *****************************************************************************/
#include "cosm/repr/losQ3D.hpp"

#include "cosm/ds/cell3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
losQ3D::losQ3D(const const_grid_view& c_view)
    : base_los(c_view),
      ER_CLIENT_INIT("cosm.repr.losQ3D") {
  ER_ASSERT(1 == view().shape()[2], "Q3D view does not have zsize=1");
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
const cds::cell3D& losQ3D::access(size_t i, size_t j) const {
  ER_ASSERT(i < xsize(),
            "Out of bounds X access: %zu >= %lu",
            i,
            xsize());
  ER_ASSERT(j < ysize(),
            "Out of bounds Y access: %zu >= %lu",
            j,
            ysize());
  return view()[i][j][0];
} /* access() */

NS_END(repr, cosm);
