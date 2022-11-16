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
NS_START(cosm, arena);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief The status return from \ref base_arena_map::update() (or the overriden
 * version in a derived class).
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

NS_END(arena, cosm);
