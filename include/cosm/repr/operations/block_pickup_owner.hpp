/**
 * \file block_pickup_owner.hpp
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
namespace cosm::repr::operations {

/*******************************************************************************
 * Type Definitions
 ******************************************************************************/
/**
 * Specify who is the owner of the block being picked up during a block picked
 * event; block state is updated differently depending on who the owner is.
 */
enum block_pickup_owner {
    ekARENA_MAP,
    ekROBOT
  };


} /* namespace cosm::repr::operations */

