/**
 * \file spatial_entity3D.hpp
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

#include "cosm/repr/spatial_entity2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {

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
  rspatial::euclidean_dist zrsize(void) const {
    return rspatial::euclidean_dist(rbb().zsize());
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

} /* namespace cosm::repr */
