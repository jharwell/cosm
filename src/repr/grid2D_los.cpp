/**
 * \file grid2D_los.cpp
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
#include "cosm/repr/grid2D_los.hpp"
#include "cosm/ds/cell2D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
grid2D_los::grid2D_los(const rtypes::type_uuid& c_id,
           const grid_view_type& c_view,
           const rtypes::discretize_ratio& c_resolution)
    : base_grid_los(c_id, c_view, c_resolution),
      ER_CLIENT_INIT("cosm.repr.grid2D_los") {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
grid2D_los::field_coord_dtype grid2D_los::abs_ll(void) const {
  return access(0, 0).loc();
}
grid2D_los::field_coord_dtype grid2D_los::abs_ul(void) const {
  return access(0, ydsize() - 1).loc();
}
grid2D_los::field_coord_dtype grid2D_los::abs_lr(void) const {
  return access(xdsize() - 1, 0).loc();
}
grid2D_los::field_coord_dtype grid2D_los::abs_ur(void) const {
  return access(xdsize() - 1, ydsize() - 1).loc();
}


NS_END(repr, cosm);