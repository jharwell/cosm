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
    : base_los(c_view), ER_CLIENT_INIT("cosm.repr.losQ3D") {
  ER_ASSERT(1 == view().shape()[2], "Q3D view does not have zsize=1");
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
const cds::cell3D& losQ3D::access(size_t i, size_t j) const {
  ER_ASSERT(i < xsize(), "Out of bounds X access: %zu >= %lu", i, xsize());
  ER_ASSERT(j < ysize(), "Out of bounds Y access: %zu >= %lu", j, ysize());
  return view()[i][j][0];
} /* access() */

bool losQ3D::contains_abs(const rmath::vector3z& loc) const {
    for (size_t i = 0; i < xsize(); ++i) {
      for (size_t j = 0; j < ysize(); ++j) {
        if (access(i, j).loc() == loc) {
          return true;
        }
      } /* for(j..) */
    }   /* for(i..) */
  return false;
} /* contains_abs() */

bool losQ3D::contains_rel(const rmath::vector2z& loc) const {
  return (loc.x() < xsize()) && (loc.y() < ysize());
} /* contains_rel() */

rmath::vector3z losQ3D::abs_ll(void) const {
  return access(0, 0).loc();
} /* abs_ll() */

rmath::vector3z losQ3D::abs_ul(void) const {
  return access(0, ysize() - 1).loc();
} /* abs_ul() */

rmath::vector3z losQ3D::abs_lr(void) const {
  return access(xsize() - 1, 0).loc();
} /* abs_lr() */

rmath::vector3z losQ3D::abs_ur(void) const {
  return access(xsize() - 1, ysize() - 1).loc();
} /* abs_ur() */

NS_END(repr, cosm);
