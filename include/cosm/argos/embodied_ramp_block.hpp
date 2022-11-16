/**
 * \file embodied_ramp_block.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>
#include <memory>
#include <utility>

#include "cosm/argos/ramp_block_embodiment.hpp"
#include "cosm/hal/hal.hpp"
#include "cosm/repr/embodied_entity.hpp"
#include "cosm/repr/ramp_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, argos);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class embodied_ramp_block
 * \ingroup argos
 *
 * \brief A \ref crepr::ramp_block3D + ARGoS embodiment.
 */
class embodied_ramp_block final
    : public crepr::ramp_block3D,
      public repr::embodied_entity<ramp_block_embodiment> {
 public:
  using embodiment_type = ramp_block_embodiment;

  embodied_ramp_block(const rtypes::type_uuid& id,
                      const rmath::vector3d& dim,
                      const rtypes::discretize_ratio& arena_res,
                      std::unique_ptr<ramp_block_embodiment> embodiment)
      : ramp_block3D(id, dim, arena_res),
        embodied_entity(std::move(embodiment)) {}
  ~embodied_ramp_block(void) override = default;
};

NS_END(argos, cosm);
