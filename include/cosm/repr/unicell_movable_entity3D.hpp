/**
 * \file unicell_movable_entity3D.hpp
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

#ifndef INCLUDE_COSM_REPR_UNICELL_MOVABLE_ENTITY3D_HPP_
#define INCLUDE_COSM_REPR_UNICELL_MOVABLE_ENTITY3D_HPP_

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
 * \ingroup cosm repr
 *
 * \brief A class representing 3D objects that reside within a 3D grid whose
 * position CAN change during the lifetime of the object.
 */
class unicell_movable_entity3D : public unicell_entity3D {
 public:
  using unicell_entity3D::dpos3D;
  using unicell_entity3D::rpos3D;
  using unicell_entity3D::unicell_entity3D;

  static constexpr bool is_movable(void) { return true; }

  ~unicell_movable_entity3D(void) override = default;

  void rpos3D(const rmath::vector3d& pos) {
    unicell_entity3D::rpos3D<unicell_movable_entity3D>(pos);
  }
  void dpos3D(const rmath::vector3z& pos) {
    unicell_entity3D::dpos3D<unicell_movable_entity3D>(pos);
  }
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_UNICELL_MOVABLE_ENTITY3D_HPP_ */
