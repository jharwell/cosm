/**
 * \file cube_block3D.hpp
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

#ifndef INCLUDE_COSM_REPR_CUBE_BLOCK3D_HPP_
#define INCLUDE_COSM_REPR_CUBE_BLOCK3D_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <utility>

#include "cosm/repr/base_block3D.hpp"
#include "cosm/hal/hal.hpp"

#if COSM_HAL_TARGET == HAL_TARGET_ARGOS_FOOTBOT
#include <boost/optional.hpp>
#include "cosm/repr/embodied_block.hpp"
#endif

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class cube_block3D
 * \ingroup cosm repr
 *
 * \brief A 3D representation of a cubical block within the arena. Bounding box
 * for cube blocks is 1x1x1 cells in 3D.
 */
class cube_block3D final : public base_block3D {
 public:
  explicit cube_block3D(const rmath::vector3d& dim)
      : base_block3D(dim,
                   rutils::color::kBLACK,
                   crepr::block_type::ekCUBE,
                   rtypes::constants::kNoUUID) {}

  cube_block3D(const rmath::vector3d& dim, const rtypes::type_uuid& id) noexcept
      : base_block3D(dim, rutils::color::kBLACK, crepr::block_type::ekCUBE, id) {}

  std::unique_ptr<base_block3D> clone(void) const override {
    auto tmp = std::make_unique<cube_block3D>(dims3D(), id());

    /* copy core definition features */
    tmp->dpos3D(this->dpos3D());
    tmp->rpos3D(this->rpos3D());

    /* copy metadata */
    tmp->md()->robot_id_reset();
    tmp->md()->metrics_copy(this->md());
    return tmp;
  } /* clone() */

#if COSM_HAL_TARGET == HAL_TARGET_ARGOS_FOOTBOT
  const boost::optional<embodied_cube_block>& embodiment(void) const {
    return m_embodiment;
  }
  void embodiment(boost::optional<embodied_cube_block>&& b) {
    m_embodiment = std::move(b);
  }
#endif

 private:
#if COSM_HAL_TARGET == HAL_TARGET_ARGOS_FOOTBOT
  /* clang-format off */
  boost::optional<embodied_cube_block> m_embodiment{};
  /* clang-format on */
#endif
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_CUBE_BLOCK3D_HPP_ */
