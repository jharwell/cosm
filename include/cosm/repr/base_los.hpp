/**
 * \file base_los.hpp
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

#ifndef INCLUDE_COSM_REPR_BASE_LOS_HPP_
#define INCLUDE_COSM_REPR_BASE_LOS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>

#include "rcppsw/er/client.hpp"
#include "rcppsw/types/discretize_ratio.hpp"
#include "rcppsw/types/type_uuid.hpp"
#include "rcppsw/math/vector2.hpp"

#include "cosm/cosm.hpp"
#include "cosm/repr/base_grid_view_entity.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_los
 * \ingroup repr
 *
 * \brief A repr of the robot's current line-of-sight. The robot is only able
 * to update its internal state based on the information present in the
 * per-timestep updates to this object.
 *
 * The LOS for a robot is always square UNLESS the robot is near the edge of the
 * source field, and a square grid would result in out-of-bounds array
 * accesses. In that case, a truncated LOS is created.
 *
 * All coordinates within a LOS are relative to the LOS itself (not its location
 * within the arena). The origin is in the lower left corner of the LOS,
 * regardless of the orientation of the robot/LOS in the arena.
 */
template <typename TGridViewEntityType, typename TFieldCoordRType>
class base_los : public rer::client<base_los<TGridViewEntityType,
                                             TFieldCoordRType>>,
                 public TGridViewEntityType {
 public:
  using field_coord_rtype = TFieldCoordRType;
  using los_coord_type = rmath::vector2z;
  using grid_view_entity_type = TGridViewEntityType;

  using typename grid_view_entity_type::grid_type;
  using typename grid_view_entity_type::cell_type;
  using typename grid_view_entity_type::grid_view_type;
  using field_coord_dtype = typename grid_view_entity_type::coord_type;

  using grid_view_entity_type::xdsize;
  using grid_view_entity_type::ydsize;
  using grid_view_entity_type::access;

  virtual field_coord_dtype abs_ll(void) const = 0;
  virtual field_coord_dtype abs_ul(void) const = 0;
  virtual field_coord_dtype abs_lr(void) const = 0;
  virtual field_coord_dtype abs_ur(void) const = 0;

  base_los(const rtypes::type_uuid& c_id,
           const grid_view_type& c_view,
           const rtypes::discretize_ratio& c_resolution)
      : ER_CLIENT_INIT("cosm.repr.base_los"),
        grid_view_entity_type(c_id, c_view, c_resolution) {}
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_BASE_LOS_HPP_ */
