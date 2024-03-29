/**
 * \file free_blocks_calculator.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/arena/ds/cache_vector.hpp"
#include "cosm/cosm.hpp"
#include "cosm/ds/block3D_vector.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::arena {

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
  explicit free_blocks_calculator(bool oos_ok) : mc_oos_ok(oos_ok) {}

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

 private:
  /* clang-format off */
  const bool mc_oos_ok;
  /* clang-format on */
};

} /* namespace cosm::arena */
