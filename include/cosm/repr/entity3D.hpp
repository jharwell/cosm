/**
 * \file entity3D.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

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
 * \ingroup repr
 *
 * \brief Base class from which all arena spatial entities which can be
 * represented in 3D derive.
 */
class entity3D : public spatial_entity3D {
 public:
  using spatial_entity3D::spatial_entity3D;

  ~entity3D(void) override = default;

  rmath::vector3d ranchor3D(void) const { return rbb().anchor3D(); }
  rmath::vector3z danchor3D(void) const { return dbb().anchor3D(); }
  rmath::vector2d ranchor2D(void) const { return ranchor3D().to_2D(); }
  rmath::vector2z danchor2D(void) const { return danchor3D().to_2D(); }

  rmath::vector3d rcenter3D(void) const { return rbb().center3D(); }
  rmath::vector3z dcenter3D(void) const { return dbb().center3D(); }
  rmath::vector2d rcenter2D(void) const { return rcenter3D().to_2D(); }
  rmath::vector2z dcenter2D(void) const { return dcenter3D().to_2D(); }

  rmath::vector3d rdims3D(void) const { return rbb().dims3D(); }
  rmath::vector3z ddims3D(void) const { return dbb().dims3D(); }
  rmath::vector2d rdims2D(void) const { return rdims3D().to_2D(); }
  rmath::vector2z ddims2D(void) const { return ddims3D().to_2D(); }

  entity_dimensionality dimensionality(void) const override final {
    return entity_dimensionality::ek3D;
  }
};

NS_END(repr, cosm);
