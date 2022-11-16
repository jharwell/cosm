/**
 * \file gridQ3D_los.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 *****************************************************************************/
#include "cosm/repr/gridQ3D_los.hpp"

#include "cosm/ds/cell3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
gridQ3D_los::gridQ3D_los(const rtypes::type_uuid& c_id,
                         const grid_view_type& c_view,
                         const rtypes::discretize_ratio& c_resolution)
    : base_grid_los(c_id, c_view, c_resolution),
      ER_CLIENT_INIT("cosm.repr.gridQ3D_los") {
  ER_ASSERT(1 == zdsize(), "Q3D view does not have zsize=1");
}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
gridQ3D_los::field_coord_dtype gridQ3D_los::abs_ll(void) const {
  return access(0, 0, 0).loc();
}
gridQ3D_los::field_coord_dtype gridQ3D_los::abs_ul(void) const {
  return access(0, ydsize() - 1, 0).loc();
}
gridQ3D_los::field_coord_dtype gridQ3D_los::abs_lr(void) const {
  return access(xdsize() - 1, 0, 0).loc();
}
gridQ3D_los::field_coord_dtype gridQ3D_los::abs_ur(void) const {
  return access(xdsize() - 1, ydsize() - 1, 0).loc();
}

NS_END(repr, cosm);
