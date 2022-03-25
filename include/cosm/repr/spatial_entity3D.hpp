/**
 * \file spatial_entity3D.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "rcppsw/math/vector3.hpp"

#include "cosm/repr/spatial_entity2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class spatial_entity3D
 * \ingroup repr
 *
 * \brief Base class from which all arena spatial entities which can be
 * represented in 3D derive. Basically defines the interface for a 2D bounding
 * box for entities.
 */
class spatial_entity3D : public spatial_entity2D {
 public:
  using spatial_entity2D::spatial_entity2D;

  ~spatial_entity3D(void) override = default;

  /**
   * \brief Calculate the span in Z of a 3D entity given its location and
   * dimension in Z.
   *
   * \return The span in Z of the entity.
   */
  rmath::ranged zrspan(void) const { return rbb().xspan(); }

  /**
   * \brief Get the size of the 3D entity in the Z direction in real
   * coordinates.
   */
  rtypes::spatial_dist zrsize(void) const {
    return rtypes::spatial_dist(rbb().zsize());
  }

  /**
   * \brief Calculate the span in Z of the entity in discrete coordinates.
   */
  rmath::rangez zdspan(void) const { return dbb().zspan(); }

  /**
   * \brief Get the size of the 3D entity in the Z direction in discrete
   * coordinates.
   */
  size_t zdsize(void) const { return dbb().zsize(); }
};

NS_END(repr, cosm);

