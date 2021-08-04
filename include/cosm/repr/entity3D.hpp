/**
 * \file entity3D.hpp
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

#ifndef INCLUDE_COSM_REPR_ENTITY3D_HPP_
#define INCLUDE_COSM_REPR_ENTITY3D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector3.hpp"

#include "cosm/cosm.hpp"
#include "cosm/repr/spatial_entity3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class entity3D
 * \ingroup cosm repr
 *
 * \brief Base class from which all arena spatial entities which can be
 * represented in 3D derive.
 */
class entity3D : public spatial_entity3D {
 public:
  using spatial_entity3D::spatial_entity3D;

  ~entity3D(void) override = default;

  virtual rmath::vector2d rcenter2D(void) const = 0;
  virtual rmath::vector2d ranchor2D(void) const = 0;
  virtual rmath::vector3d rcenter3D(void) const = 0;
  virtual rmath::vector3d ranchor3D(void) const = 0;

  virtual rmath::vector2z dcenter2D(void) const = 0;
  virtual rmath::vector2z danchor2D(void) const = 0;
  virtual rmath::vector3z dcenter3D(void) const = 0;
  virtual rmath::vector3z danchor3D(void) const = 0;

  entity_dimensionality dimensionality(void) const override final {
    return entity_dimensionality::ek3D;
  }
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_ENTITY3D_HPP_ */
