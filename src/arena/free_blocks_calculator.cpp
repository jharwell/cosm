/**
 * \file free_blocks_calculator.cpp
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
NS_START(cosm, arena);

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

NS_END(arena, cosm);
