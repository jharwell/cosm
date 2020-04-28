/**
 * \file base_los.cpp
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
#include "cosm/repr/base_los.hpp"

#include "cosm/ds/cell2D.hpp"
#include "cosm/ds/cell3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
template<typename TCellType>
bool base_los<TCellType>::contains_loc(const abs_coord_type& loc) const {
  for (size_t i = 0; i < xsize(); ++i) {
    for (size_t j = 0; j < ysize(); ++j) {
      if (access(i, j).loc() == loc) {
        return true;
      }
    } /* for(j..) */
  }   /* for(i..) */
  return false;
} /* contains_loc() */

template<typename TCellType>
typename base_los<TCellType>::abs_coord_type base_los<TCellType>::abs_ll(void) const {
  return access(0, 0).loc();
} /* abs_ll() */

template<typename TCellType>
typename base_los<TCellType>::abs_coord_type base_los<TCellType>::abs_ul(void) const {
  return access(0, ysize() - 1).loc();
} /* abs_ul() */

template<typename TCellType>
typename base_los<TCellType>::abs_coord_type base_los<TCellType>::abs_lr(void) const {
  return access(xsize() - 1, 0).loc();
} /* abs_lr() */

template<typename TCellType>
typename base_los<TCellType>::abs_coord_type base_los<TCellType>::abs_ur(void) const {
  return access(xsize() - 1, ysize() - 1).loc();
} /* abs_ur() */

/*******************************************************************************
 * Template Instantiations
 ******************************************************************************/
template class base_los<cds::cell2D>;
template class base_los<cds::cell3D>;

NS_END(repr, cosm);
