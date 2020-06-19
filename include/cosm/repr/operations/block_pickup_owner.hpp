/**
 * \file block_pickup_owner.hpp
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

#ifndef INCLUDE_COSM_REPR_OPERATIONS_BLOCK_PICKUP_OWNER_HPP_
#define INCLUDE_COSM_REPR_OPERATIONS_BLOCK_PICKUP_OWNER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, repr, operations);

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


NS_END(operations, repr, cosm);

#endif /* INCLUDE_COSM_REPR_OPERATIONS_BLOCK_PICKUP_OWNER_HPP_ */
