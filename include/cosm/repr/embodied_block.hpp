/**
 * \file embodied_block.hpp
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

#ifndef INCLUDE_COSM_REPR_EMBODIED_BLOCK_HPP_
#define INCLUDE_COSM_REPR_EMBODIED_BLOCK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/variant.hpp>

#include "cosm/cosm.hpp"
#include "cosm/hal/hal.hpp"

#if (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT) || (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D)
#include <argos3/plugins/simulator/entities/box_entity.h>
#endif

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
static_assert((COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT) ||
              (COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D),
              "Embodied blocks can only be used in ARGoS");
/**
 * \struct embodied_cube_block
 *
 * \brief Handle for the implementation on how to take a \ref cube_block3D and
 * make it physically embodied in the 3D world in ARGoS.
 *
 * Have to use raw pointers and new in order to be able to hand ownership of the
 * block to ARGoS.
 */
struct embodied_cube_block {
  argos::CBoxEntity* box{ nullptr };
};

/**
 * \struct embodied_ramp_block
 *
 * \brief Handle for the implementation on how to take a \ref ramp_block3D and
 * make it physically embodied in the 3D world in ARGoS.
 *
 * Right now, this is done by using very thin ARGoS boxes for the slope (top),
 * the back, and the botton of the block. Since the sides are triangles, that is
 * not directly supported by ARGoS, and I don't want to implement it
 *
 * Have to use raw pointers and new in order to be able to hand ownership of the
 * block to ARGoS.
 */
struct embodied_ramp_block {
  argos::CBoxEntity* top{ nullptr };
  argos::CBoxEntity* bottom{ nullptr };
  argos::CBoxEntity* back{ nullptr };
};

using embodied_block_variant =
    boost::variant<embodied_cube_block, embodied_ramp_block>;

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_EMBODIED_BLOCK_HPP_ */
