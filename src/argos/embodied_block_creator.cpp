/**
 * \file embodied_block_creator.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/argos/embodied_block_creator.hpp"

#include "cosm/arena/base_arena_map.hpp"
#include "cosm/argos/block_embodiment_creator.hpp"
#include "cosm/pal/argos/swarm_manager_adaptor.hpp"
#include "cosm/repr/cube_block3D.hpp"
#include "cosm/repr/ramp_block3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, argos);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
cargos::embodied_block_varianto
embodied_block_creator::operator()(const crepr::cube_block3D* block) const {
  auto embodied = std::make_unique<embodied_cube_block>(
      block->id(),
      block->rdims3D(),
      const_cast<const cpargos::swarm_manager_adaptor*>(m_sm)
          ->arena_map()
          ->grid_resolution(),
      nullptr);
  return { std::move(embodied) };
}

cargos::embodied_block_varianto
embodied_block_creator::operator()(const crepr::ramp_block3D* block) const {
  auto embodied = std::make_unique<embodied_ramp_block>(
      block->id(),
      block->rdims3D(),
      const_cast<const cpargos::swarm_manager_adaptor*>(m_sm)
          ->arena_map()
          ->grid_resolution(),
      nullptr);
  return { std::move(embodied) };
}

NS_END(argos, cosm);
