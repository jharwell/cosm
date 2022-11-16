/**
 * \file cube_block3D.hpp
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
 * \class cube_block3D
 * \ingroup repr
 *
 * \brief A 3D representation of a cubical block within the arena. Bounding box
 * for cube blocks is 1x1x1 cells in 3D.
 */
class cube_block3D : public sim_block3D {
 public:
  cube_block3D(const rtypes::type_uuid& id,
               const rmath::vector3d& dim,
               const rtypes::discretize_ratio& arena_res) noexcept
      : sim_block3D(id,
                    dim,
                    arena_res,
                    rutils::color::kBLACK,
                    crepr::block_type::ekCUBE) {}

  std::unique_ptr<base_block3D> clone(void) const override {
    auto tmp = std::make_unique<cube_block3D>(id(), rdims3D(), arena_res());
    this->sim_block3D::clone_impl(tmp.get());
    return tmp;
  } /* clone() */
};
NS_END(repr, cosm);
