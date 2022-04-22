/**
 * \file cube_block_embodiment.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/plugins/simulator/entities/box_entity.h>

#include "cosm/repr/base_embodiment.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, argos);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/

/**
 * \struct cube_block_embodiment
 * \ingroup argos
 *
 * \brief Handle for the implementation on how to take a \ref cube_block3D and
 * make it physically embodied in the 3D world in ARGoS.
 *
 * Have to use raw pointers and new in order to be able to hand ownership of the
 * block to ARGoS.
 */
struct cube_block_embodiment : public crepr::base_embodiment {
  ::argos::CBoxEntity* box{ nullptr };
};

NS_END(argos, cosm);
