/**
 * \file free_blocks_calculator.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/arena/free_blocks_calculator.hpp"

#include <algorithm>

#include "rcppsw/types/type_uuid.hpp"

#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::arena {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
cds::block3D_vectorno free_blocks_calculator::operator()(
    const cds::block3D_vectorno& c_all_blocks,
    const cads::acache_vectorro& c_all_caches) const {
  cds::block3D_vectorno free_blocks;
  std::copy_if(c_all_blocks.begin(),
               c_all_blocks.end(),
               std::back_inserter(free_blocks),
               [&](const auto& b) RCPPSW_PURE {
                 /* block not carried by robot */
                 return !b->is_carried_by_robot() &&
                        /*
                      * Block is not out of sight. Handles cases with powerlaw
                      * distribution where block dropped in the nest fail to
                      * re-distribute and are currently pending to be
                      * re-distributed later (maybe) if arena conditions allow
                      * it (e.g., enough robots have picked up blocks so there
                      * is space in a cluster for the block). See COSM#124. See
                      * also COSM#142 for correctly computing block cluster
                      * locations during deferred arena map initialization.
                      */
                        (mc_oos_ok || !b->is_out_of_sight()) &&
                        /*
                      * Block not inside cache (to catch blocks that were on the
                      * host cell for the cache, and we incorporated into it
                      * during creation).
                      */
                        std::all_of(c_all_caches.begin(),
                                    c_all_caches.end(),
                                    [&](const auto& c) {
                                      return !c->contains_block(b);
                                    });
               });
  return free_blocks;
} /* operator()() */

} /* namespace cosm::arena */
