/**
 * \file base_grid_los.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/types/discretize_ratio.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/repr/base_grid_view_entity.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_grid_los
 * \ingroup repr
 *
 * \brief A representation of the robot's current line-of-sight as it moves
 * through a grid of some kind. The robot is only able to update its internal
 * state based on the information present in the per-timestep updates to this
 * object.
 *
 * The LOS for a robot is always square UNLESS the robot is near the edge of the
 * source field, and a square grid would result in out-of-bounds array
 * accesses. In that case, a truncated LOS is created.
 *
 * All coordinates within a LOS are relative to the LOS itself (not its location
 * within the arena). The origin is in the lower left corner of the LOS,
 * regardless of the orientation of the robot.
 */
template <typename TGridViewEntityType, typename TFieldCoordRType>
class base_grid_los
    : public rer::client<base_grid_los<TGridViewEntityType, TFieldCoordRType>>,
      public TGridViewEntityType {
 public:
  using field_coord_rtype = TFieldCoordRType;
  using los_coord_type = rmath::vector2z;
  using grid_view_entity_type = TGridViewEntityType;

  using typename grid_view_entity_type::cell_type;
  using typename grid_view_entity_type::grid_type;
  using typename grid_view_entity_type::grid_view_type;
  using field_coord_dtype = typename grid_view_entity_type::coord_type;

  using grid_view_entity_type::access;
  using grid_view_entity_type::xdsize;
  using grid_view_entity_type::ydsize;

  virtual field_coord_dtype abs_ll(void) const = 0;
  virtual field_coord_dtype abs_ul(void) const = 0;
  virtual field_coord_dtype abs_lr(void) const = 0;
  virtual field_coord_dtype abs_ur(void) const = 0;

  base_grid_los(const rtypes::type_uuid& c_id,
                const grid_view_type& c_view,
                const rtypes::discretize_ratio& c_resolution)
      : ER_CLIENT_INIT("cosm.repr.base_grid_los"),
        grid_view_entity_type(c_id, c_view, c_resolution) {}
};

NS_END(repr, cosm);
