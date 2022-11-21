/**
 * \file block_carrying_controller.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/controller/block_carrying_controller.hpp"

#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::controller {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_carrying_controller::~block_carrying_controller(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_carrying_controller::reset(void) { m_block.reset(); }

void block_carrying_controller::block(std::unique_ptr<crepr::base_block3D> block) {
  m_block = std::move(block);
}

std::unique_ptr<crepr::base_block3D>
block_carrying_controller::block_release(void) {
  return std::move(m_block);
}

} /* namespace cosm::controller */
