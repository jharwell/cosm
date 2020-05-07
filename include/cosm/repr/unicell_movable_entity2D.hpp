/**
 * \file unicell_movable_entity2D.hpp
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

#ifndef INCLUDE_COSM_REPR_UNICELL_MOVABLE_ENTITY2D_HPP_
#define INCLUDE_COSM_REPR_UNICELL_MOVABLE_ENTITY2D_HPP_

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
 * \ingroup cosm repr
 *
 * \brief A class representing 2D objects that reside within one or more squares
 * within a 2D grid whose position CAN change during the lifetime of the object.
 */
class unicell_movable_entity2D : public unicell_entity2D {
 public:
  using unicell_entity2D::dpos2D;
  using unicell_entity2D::rpos2D;
  using unicell_entity2D::unicell_entity2D;

  static constexpr bool is_movable(void) { return true; }

  ~unicell_movable_entity2D(void) override = default;

  void rpos2D(const rmath::vector2d& pos) {
    unicell_entity2D::rpos2D<unicell_movable_entity2D>(pos);
  }
  void dpos2D(const rmath::vector2z& pos) {
    unicell_entity2D::dpos2D<unicell_movable_entity2D>(pos);
  }
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_UNICELL_MOVABLE_ENTITY2D_HPP_ */
