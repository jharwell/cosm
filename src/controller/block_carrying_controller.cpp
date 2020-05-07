/**
 * \file block_carrying_controller.cpp
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
#include "cosm/controller/block_carrying_controller.hpp"

#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, controller);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_carrying_controller::reset(void) { m_block.reset(); }

void block_carrying_controller::block(std::unique_ptr<crepr::base_block3D> block) {
  m_block = std::move(block);
}

std::unique_ptr<crepr::base_block3D> block_carrying_controller::block_release(void) {
  return std::move(m_block);
}

NS_END(controller, cosm);
