/**
 * \file embodied_block_creator.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/argos/embodied_block_creator.hpp"

#include "cosm/pal/argos/swarm_manager_adaptor.hpp"
#include "cosm/repr/cube_block3D.hpp"
#include "cosm/repr/ramp_block3D.hpp"
#include "cosm/argos/block_embodiment_creator.hpp"
#include "cosm/arena/base_arena_map.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, argos);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
cargos::embodied_block_varianto
embodied_block_creator::operator()(const crepr::cube_block3D* block) const {
  auto embodied = std::make_unique<embodied_cube_block>(block->id(),
                                                        block->rdims3D(),
                                                        const_cast<const cpargos::swarm_manager_adaptor*>(m_sm)->arena_map()->grid_resolution(),
                                                        nullptr);
  return {std::move(embodied)};
}

cargos::embodied_block_varianto
embodied_block_creator::operator()(const crepr::ramp_block3D* block) const {
  auto embodied = std::make_unique<embodied_ramp_block>(block->id(),
                                                        block->rdims3D(),
                                                        const_cast<const cpargos::swarm_manager_adaptor*>(m_sm)->arena_map()->grid_resolution(),
                                                        nullptr);
  return {std::move(embodied)};
}

NS_END(argos, cosm);