/**
 * \file ramp_block3D.hpp
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

#ifndef INCLUDE_COSM_REPR_RAMP_BLOCK3D_HPP_
#define INCLUDE_COSM_REPR_RAMP_BLOCK3D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "cosm/repr/base_block.hpp"
#include "cosm/repr/unicell_movable_entity3D.hpp"

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
 * \brief A 3D representation of a ramp block within the arena. The bounding box
 * for ramp blocks is 2x1x1 cells in 3D.
 */
class ramp_block3D final : public base_block<unicell_movable_entity3D> {
 public:
  explicit ramp_block3D(const rmath::vector3d& dim)
      : base_block(dim,
                   rutils::color::kBLUE,
                   crepr::block_type::ekRAMP,
                   rtypes::constants::kNoUUID) {}

  ramp_block3D(const rmath::vector3d& dim, const rtypes::type_uuid& id) noexcept
      : base_block(dim,
                   rutils::color::kBLUE,
                   crepr::block_type::ekRAMP,
                   id) {}

  std::unique_ptr<base_block> clone(void) const override {
    auto tmp = std::make_unique<ramp_block3D>(dims(), id());

    /* copy core definition features */
    tmp->dloc(this->dloc());
    tmp->rloc(this->rloc());

    /* copy metadata */
    tmp->md()->robot_id_reset();
    tmp->md()->metrics_copy(this->md());
    return tmp;
  } /* clone() */
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_RAMP_BLOCK3D_HPP_ */
