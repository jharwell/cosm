/**
 * \file los2d.hpp
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

#ifndef INCLUDE_COSM_REPR_LOS2D_HPP_
#define INCLUDE_COSM_REPR_LOS2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/ds/base_grid2D.hpp"

#include "cosm/repr/base_los.hpp"
#include "cosm/repr/grid2D_view_entity.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ds {
class cell2D;
} /* namespace cosm::ds */

NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class los2D
 * \ingroup repr
 *
 * \brief A repr of the robot's current line-of-sight in 2D.
 */
class los2D : public crepr::base_los<
  grid2D_view_entity<rds::base_grid2D<cds::cell2D>,
                     rds::base_grid2D<cds::cell2D>::const_grid_view>,
  rmath::vector2d>,
              public rer::client<los2D> {
 public:
  using los_type = crepr::base_los<grid2D_view_entity<rds::base_grid2D<cds::cell2D>,
                                                      rds::base_grid2D<cds::cell2D>::const_grid_view>,
                                   rmath::vector2d>;
  using los_type::grid_view_type;
  using los_type::access;


  los2D(const rtypes::type_uuid& c_id,
        const grid_view_type& c_view,
        const rtypes::discretize_ratio& c_resolution);

  field_coord_dtype abs_ll(void) const override;
  field_coord_dtype abs_ul(void) const override;
  field_coord_dtype abs_lr(void) const override;
  field_coord_dtype abs_ur(void) const override;
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_LOS2D_HPP_ */
