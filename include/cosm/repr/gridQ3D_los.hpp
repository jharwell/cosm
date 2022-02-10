/**
 * \file gridQ3D_los.hpp
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
#include "rcppsw/ds/grid3D.hpp"
#include "rcppsw/math/vector3.hpp"

#include "cosm/repr/base_grid_los.hpp"
#include "cosm/repr/grid3D_view_entity.hpp"
#include "cosm/ds/cell3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class gridQ3D_los
 * \ingroup repr
 *
 * \brief A repr of the robot's current line-of-sight in quasi-3D. "Quasi"
 * because the LOS object itself is only 2D, BUT contains information about a
 * slice of 3D cells. This is in keeping with making the robot controllers as
 * simple as poossible.
 */
class gridQ3D_los : public crepr::base_grid_los<
  grid3D_view_entity<rds::grid3D<cds::cell3D>,
                     rds::grid3D<cds::cell3D>::const_grid_view>,
  rmath::vector3d>,
               public rer::client<gridQ3D_los> {
 public:
  using los_type = crepr::base_grid_los<grid3D_view_entity<rds::grid3D<cds::cell3D>,
                                                           rds::grid3D<cds::cell3D>::const_grid_view>,
                                   rmath::vector3d>;
  using los_type::grid_view_type;
  using los_type::access;

  gridQ3D_los(const rtypes::type_uuid& c_id,
              const grid_view_type& c_view,
              const rtypes::discretize_ratio& c_resolution);

  field_coord_dtype abs_ll(void) const override;
  field_coord_dtype abs_ul(void) const override;
  field_coord_dtype abs_lr(void) const override;
  field_coord_dtype abs_ur(void) const override;
};

NS_END(repr, cosm);

