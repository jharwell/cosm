/**
 * \file block_detect_context.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, controller);

/*******************************************************************************
 * Enum Definitions
 ******************************************************************************/
/**
 * \brief The calling context when something queries a \ref
 * block_carrying_controller if it has detected a block.
 */
enum class block_detect_context {
    /**
     * \brief Part of the robot controller
     */
    ekROBOT,

    /**
     * \brief Part of arena/loop functions
     */
    ekARENA
  };

NS_END(controller, cosm);
