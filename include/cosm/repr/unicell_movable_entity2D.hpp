/**
 * \file unicell_movable_entity2D.hpp
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
#include "cosm/repr/unicell_entity2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class unicell_movable_entity2D
 * \ingroup repr
 *
 * \brief A class representing 2D objects that reside within one or more squares
 * within a 2D grid whose position CAN change during the lifetime of the object.
 */
class unicell_movable_entity2D : public unicell_entity2D {
 public:
  using unicell_entity2D::danchor2D;
  using unicell_entity2D::ranchor2D;
  using unicell_entity2D::unicell_entity2D;

  static constexpr bool is_movable(void) { return true; }

  ~unicell_movable_entity2D(void) override = default;

  void ranchor2D(const rmath::vector2d& anchor) {
    unicell_entity2D::ranchor2D<unicell_movable_entity2D>(anchor);
  }
  void danchor2D(const rmath::vector2z& anchor) {
    unicell_entity2D::danchor2D<unicell_movable_entity2D>(anchor);
  }
};

NS_END(repr, cosm);
