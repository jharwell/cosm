/**
 * \file grid2D_los.hpp
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
#include "rcppsw/ds/grid2D.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/ds/cell2D.hpp"
#include "cosm/repr/base_grid_los.hpp"
#include "cosm/repr/grid2D_view_entity.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class grid2D_los
 * \ingroup repr
 *
 * \brief A repr of the robot's current line-of-sight in 2D.
 */
class grid2D_los
    : public crepr::base_grid_los<
          grid2D_view_entity<rds::grid2D<cds::cell2D>,
                             rds::grid2D<cds::cell2D>::const_grid_view>,
          rmath::vector2d>,
      public rer::client<grid2D_los> {
 public:
  using los_type = crepr::base_grid_los<
      grid2D_view_entity<rds::grid2D<cds::cell2D>,
                         rds::grid2D<cds::cell2D>::const_grid_view>,
      rmath::vector2d>;
  using los_type::access;
  using los_type::grid_view_type;

  grid2D_los(const rtypes::type_uuid& c_id,
             const grid_view_type& c_view,
             const rtypes::discretize_ratio& c_resolution);

  field_coord_dtype abs_ll(void) const override final;
  field_coord_dtype abs_ul(void) const override final;
  field_coord_dtype abs_lr(void) const override final;
  field_coord_dtype abs_ur(void) const override final;
};

NS_END(repr, cosm);
