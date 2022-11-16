/**
 * \file coord_search_policy.hpp
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
NS_START(cosm, foraging, block_dist);

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * \brief The policy to use when trying to find coordinates to distribute a
 * block to.
 */

enum coord_search_policy {
  /**
   * \brief Guess and check.
   */
  ekRANDOM,

  /**
   * \brief Compute a list of all the free cells in the distributable area and
   * then try each in turn to see if they will work.
   */
  ekFREE_CELL
};

NS_END(block_dist, foraging, cosm);

