/**
 * \file entity2D.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"

#include "cosm/cosm.hpp"
#include "cosm/repr/spatial_entity2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class entity2D
 * \ingroup repr
 *
 * \brief A base class from which all spatial entities which can be represented
 * in 2D derive.
 */
class entity2D : public spatial_entity2D {
 public:
  using spatial_entity2D::spatial_entity2D;

  ~entity2D(void) override = default;

  /**
   * \brief Return the anchor (LL corner) of the object in real coordinates.
   */
  rmath::vector2d ranchor2D(void) const { return rbb().anchor3D().to_2D(); }

  /**
   * \brief Return the 2D center of the object in real coordinates. This ALWAYS
   * exists, even if the \ref dcenter2D() does not.
   */
  rmath::vector2d rcenter2D(void) const { return rbb().center3D().to_2D(); }

  /**
   * \brief Return the anchor (LL corner) of the object in discrete
   * coordinates. This ALWAYS exists, evenif if \ref dcenter2D() does not.
   */
  rmath::vector2z danchor2D(void) const { return dbb().anchor3D().to_2D(); }

  /**
   * \brief Return the center of the object in discrete coordinates,
   * \a IF it exists. If the entity X,Y dimensions are both odd, then it exists,
   * otherwise, it does not.
   *
   * If this function is called on an entity which has no center, an assertion
   * should be triggered.
   */
  rmath::vector2z dcenter2D(void) const { return dbb().center3D().to_2D(); }

  rmath::vector2d rdims2D(void) const { return rbb().dims3D().to_2D(); }
  rmath::vector2z ddims2D(void) const { return dbb().dims3D().to_2D(); }

  entity_dimensionality dimensionality(void) const override final {
    return entity_dimensionality::ek2D;
  }
};

} /* namespace cosm::repr */
