/**
 * \file free_block_calculator.hpp
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

#ifndef INCLUDE_COSM_ARENA_FREE_BLOCK_CALCULATOR_HPP_
#define INCLUDE_COSM_ARENA_FREE_BLOCK_CALCULATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/arena/ds/cache_vector.hpp"
#include "cosm/cosm.hpp"
#include "cosm/ds/block3D_vector.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, arena);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class free_blocks_calculator
 * \ingroup arena
 *
 * \brief Calculates the # of free blocks in the arena, given the set of all
 * blocks and (possibly) a set of caches which might contain some of the
 * blocks.
 *
 * This calculation can't be ONLY be a member of the \ref base_arena_map and its
 * derived classes, because they have no way to calculate free blocks with a
 * *just* created set of new caches that have not yet been added to the arena
 * (pending verification after creation). A double-dispatch approach via this
 * class works for all cases.
 */
class free_blocks_calculator {
 public:
  free_blocks_calculator(void) = default;

  /* Not move/copy constructable/assignable by default */
  free_blocks_calculator(const free_blocks_calculator&) = delete;
  const free_blocks_calculator& operator=(const free_blocks_calculator&) = delete;
  free_blocks_calculator(free_blocks_calculator&&) = delete;
  free_blocks_calculator& operator=(free_blocks_calculator&&) = delete;

  cds::block3D_vectorno
  operator()(const cds::block3D_vectorno& c_all_blocks,
             const cads::acache_vectorro& c_all_caches) const;

  cds::block3D_vectorno
  operator()(const cds::block3D_vectorno& c_all_blocks) const {
    return operator()(c_all_blocks, {});
  }
};

NS_END(arena, cosm);

#endif /* INCLUDE_COSM_ARENA_FREE_BLOCK_CALCULATOR_HPP_ */
