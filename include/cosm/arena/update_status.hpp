/**
 * \file update_status.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
namespace cosm::arena {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief The status returned from \ref base_arena_map::pre_step_update() (or
 * the overriden version in a derived class).
 */
enum update_status {
  /**
   * \brief No updates actually occurred this timestep.
   */
  ekNONE,

  /**
   * \brief One or more blocks moved, requiring a redraw of the arena floor.
   */
  ekBLOCK_MOTION
};

} /* namespace cosm::arena */
