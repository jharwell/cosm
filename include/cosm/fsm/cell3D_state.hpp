/**
 * \file cell3D_state.hpp
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
NS_START(cosm, fsm);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief The state that a \ref cell3D_fsm can be in.
 */
class cell3D_state {
 public:
  enum {
    /**
     * \brief The cell's contents is unknown.
     */
    ekST_UNKNOWN,

    /**
     * \brief The cell is empty (does not hold a block or is part of a block's
     * extent).
     */
    ekST_EMPTY,

    /**
     * \brief The cell contains a block.
     */
    ekST_HAS_BLOCK,

    /**
     * \brief The cell does not contain a block, but is part of the 3D space
     * occupied by a block, in which case it also contains a reference to the
     * bock it is a part of.
     */
    ekST_BLOCK_EXTENT,
    ekST_MAX_STATES
  };
};

NS_END(cosm, fsm);
