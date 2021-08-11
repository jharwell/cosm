/**
 * \file losQ3D.hpp
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

#ifndef INCLUDE_COSM_REPR_LOSQ3D_HPP_
#define INCLUDE_COSM_REPR_LOSQ3D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/ds/base_grid3D.hpp"
#include "rcppsw/math/vector3.hpp"

#include "cosm/repr/base_los.hpp"
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
 * \class losQ3D
 * \ingroup repr
 *
 * \brief A repr of the robot's current line-of-sight in quasi-3D. "Quasi"
 * because the LOS object itself is only 2D, BUT contains information about a
 * slice of 3D cells. This is in keeping with making the robot controllers as
 * simple as poossible.
 */
class losQ3D : public crepr::base_los<
  grid3D_view_entity<rds::base_grid3D<cds::cell3D>,
                     rds::base_grid3D<cds::cell3D>::const_grid_view>,
  rmath::vector3d>,
               public rer::client<losQ3D> {
 public:
  using los_type = crepr::base_los<grid3D_view_entity<rds::base_grid3D<cds::cell3D>,
                                                      rds::base_grid3D<cds::cell3D>::const_grid_view>,
                                   rmath::vector3d>;
  using los_type::grid_view_type;
  using los_type::access;

  losQ3D(const rtypes::type_uuid& c_id,
         const grid_view_type& c_view,
         const rtypes::discretize_ratio& c_resolution);

  field_coord_dtype abs_ll(void) const override;
  field_coord_dtype abs_ul(void) const override;
  field_coord_dtype abs_lr(void) const override;
  field_coord_dtype abs_ur(void) const override;
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_LOSQ3D_HPP_ */
