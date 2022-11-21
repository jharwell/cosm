/**
 * \file ramp_block_embodiment.hpp
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
namespace cosm::argos {

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * \struct ramp_block_embodiment
 * \ingroup argos
 *
 * \brief Handle for the implementation on how to take a \ref
 * crepr::ramp_block3D and make it physically embodied in the 3D world in ARGoS.
 *
 * Right now, this is done by using very thin ARGoS boxes for the slope (top),
 * the back, and the botton of the block. Since the sides are triangles, that is
 * not directly supported by ARGoS, and I don't want to implement it
 *
 * Have to use raw pointers and new in order to be able to hand ownership of the
 * block to ARGoS.
 */
struct ramp_block_embodiment : public crepr::base_embodiment {
  ::argos::CBoxEntity* top{ nullptr };
  ::argos::CBoxEntity* bottom{ nullptr };
  ::argos::CBoxEntity* back{ nullptr };
};

} /* namespace cosm::argos */
