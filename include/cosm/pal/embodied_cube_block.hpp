/**
 * \file embodied_cube_block.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_PAL_EMBODIED_CUBE_BLOCK_HPP_
#define INCLUDE_COSM_PAL_EMBODIED_CUBE_BLOCK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <utility>

#include "cosm/hal/hal.hpp"
#include "cosm/repr/cube_block3D.hpp"
#include "cosm/pal/cube_block_embodiment.hpp"
#include "cosm/repr/embodied_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, pal);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \ingroup cosm pal
 *
 * \brief A \ref cpal::cube_block3D + ARGoS embodiment.
 */
class embodied_cube_block final : public crepr::cube_block3D,
                                  public repr::embodied_entity<cube_block_embodiment> {
 public:
  using embodiment_type = cube_block_embodiment;

  embodied_cube_block(const rtypes::type_uuid& id,
                      const rmath::vector3d& dim,
                      const rtypes::discretize_ratio& arena_res,
                      std::unique_ptr<cube_block_embodiment> embodiment)
      : cube_block3D(id, dim, arena_res),
        embodied_entity(std::move(embodiment)) {}
  ~embodied_cube_block(void) override = default;
};

NS_END(pal, cosm);

#endif /* INCLUDE_COSM_PAL_EMBODIED_CUBE_BLOCK_HPP_ */
