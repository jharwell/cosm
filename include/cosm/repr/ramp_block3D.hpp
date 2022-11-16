/**
 * \file ramp_block3D.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \ingroup repr
 *
 * \brief A 3D representation of a ramp block within the arena. The bounding box
 * for ramp blocks is 2x1x1 cells in 3D.
 */
class ramp_block3D : public sim_block3D {
 public:
  ramp_block3D(const rtypes::type_uuid& id,
               const rmath::vector3d& dim,
               const rtypes::discretize_ratio& arena_res)
      : sim_block3D(id,
                    dim,
                    arena_res,
                    rutils::color::kBLACK,
                    crepr::block_type::ekRAMP) {}

  std::unique_ptr<base_block3D> clone(void) const override {
    auto tmp = std::make_unique<ramp_block3D>(id(), rdims3D(), arena_res());
    this->sim_block3D::clone_impl(tmp.get());
    return tmp;
  } /* clone() */
};

NS_END(repr, cosm);
