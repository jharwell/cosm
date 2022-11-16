/**
 * \file cube_block_embodiment.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
