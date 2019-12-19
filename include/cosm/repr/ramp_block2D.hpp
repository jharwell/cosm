/**
 * \file ramp_block2D.hpp
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

#ifndef INCLUDE_COSM_REPR_RAMP_BLOCK2D_HPP_
#define INCLUDE_COSM_REPR_RAMP_BLOCK2D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/repr/base_block2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \ingroup cosm repr
 *
 * \brief A 2D representation of a 3D ramp block within the arena. Ramp blocks
 * are 2x1 cells in size in 2D. Ramped blocks only need X,Y dimensions, because
 * they are only handled concretely in the arena in 2D (3D is only for
 * visualization purposes, and I can cheat a bit there).
 */
class ramp_block2D final : public base_block2D {
 public:
  explicit ramp_block2D(const rmath::vector2d& dim)
      : base_block2D(dim, rutils::color::kBLUE, rtypes::constants::kNoUUID) {}

  ramp_block2D(const rmath::vector2d& dim, const rtypes::type_uuid& id) noexcept
      : base_block2D(dim, rutils::color::kBLUE, id) {}

  repr::block_type type(void) const override {
    return repr::block_type::ekRAMP;
  }
  std::unique_ptr<base_block2D> clone(void) const override {
    auto tmp = std::make_unique<ramp_block2D>(dims(), id());
    tmp->dloc(this->dloc());
    tmp->rloc(this->rloc());
    tmp->reset_robot_id();
    tmp->copy_metrics(*this);
    return tmp;
  } /* clone() */
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_RAMP_BLOCK2D_HPP_ */
