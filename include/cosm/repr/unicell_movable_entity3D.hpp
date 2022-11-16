/**
 * \file unicell_movable_entity3D.hpp
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
#include "cosm/repr/unicell_entity3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class unicell_movable_entity3D
 * \ingroup repr
 *
 * \brief A class representing 3D objects that reside within a 3D grid whose
 * position CAN change during the lifetime of the object.
 */
class unicell_movable_entity3D : public unicell_entity3D {
 public:
  using unicell_entity3D::danchor3D;
  using unicell_entity3D::ranchor3D;
  using unicell_entity3D::unicell_entity3D;

  static constexpr bool is_movable(void) { return true; }

  ~unicell_movable_entity3D(void) override = default;

  void ranchor3D(const rmath::vector3d& anchor) {
    unicell_entity3D::ranchor3D<unicell_movable_entity3D>(anchor);
  }
  void danchor3D(const rmath::vector3z& anchor) {
    unicell_entity3D::danchor3D<unicell_movable_entity3D>(anchor);
  }
};

NS_END(repr, cosm);
