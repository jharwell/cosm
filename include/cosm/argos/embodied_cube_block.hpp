/**
 * \file embodied_cube_block.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <utility>

#include "cosm/argos/cube_block_embodiment.hpp"
#include "cosm/hal/hal.hpp"
#include "cosm/repr/cube_block3D.hpp"
#include "cosm/repr/embodied_entity.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, argos);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class embodied_cube_block
 * \ingroup argos
 *
 * \brief A \ref cepr::cube_block3D + ARGoS embodiment.
 */
class embodied_cube_block final
    : public crepr::cube_block3D,
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

NS_END(argos, cosm);
