/**
 * \file block_detect_context.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::controller {

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

} /* namespace cosm::controller */
