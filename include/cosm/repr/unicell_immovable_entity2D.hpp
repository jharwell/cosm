/**
 * \file unicell_immovable_entity2D.hpp
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

#ifndef INCLUDE_COSM_REPR_UNICELL_IMMOVABLE_ENTITY2D_HPP_
#define INCLUDE_COSM_REPR_UNICELL_IMMOVABLE_ENTITY2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/types/discretize_ratio.hpp"

#include "cosm/repr/unicell_entity2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class unicell_immovable_entity2D
 * \ingroup cosm repr
 *
 * \brief A class representing 2D objects that reside within one or more squares
 * within a 2D grid whose position CANNOT change during the lifetime of the
 * object.
 */
class unicell_immovable_entity2D : public unicell_entity2D {
 public:
  static constexpr bool is_movable(void) { return false; }

  unicell_immovable_entity2D(const rtypes::type_uuid& id,
                             const rmath::vector2d& rdim,
                             const rtypes::discretize_ratio& resolution,
                             const rmath::vector2d& rcenter)
      : unicell_entity2D(id, rdim, resolution, rcenter) {}

  ~unicell_immovable_entity2D(void) override = default;
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_IMMOVABLE_UNICELL_ENTITY2D_HPP_ */
