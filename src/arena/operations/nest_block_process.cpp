/**
 * \file nest_block_process.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include "cosm/arena/operations/nest_block_process.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, operations, detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
nest_block_process::nest_block_process(crepr::base_block3D* arena_block,
                                       const rtypes::timestep& t)
    : ER_CLIENT_INIT("cosm.arena.operations.nest_block_process"),
      mc_timestep(t),
      m_arena_block(arena_block) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void nest_block_process::visit(base_arena_map& map) {
  map.ordered_lock(locking::ekNONE_HELD);
  do_visit(map);
  map.ordered_unlock(locking::ekNONE_HELD);
} /* visit() */

void nest_block_process::visit(caching_arena_map& map) {
  /*
   * We don't need the cache mutex held here, BUT we do need the other two, and
   * all mutexes always have to be acquired in the same order everywhere in
   * order to avoid deadlocks. If we let arena map the acquire the cache mutex
   * during block distribution, we can get a deadlock due to ordering.
   */
  map.ordered_lock(locking::ekNONE_HELD);
  do_visit(map);
  map.ordered_unlock(locking::ekNONE_HELD);
} /* visit() */

template <typename TArenaMap>
void nest_block_process::do_visit(TArenaMap& map) {
  /* update block after processing */
  visit(*m_arena_block);

  /* release block back into the wild */
  map.distribute_single_block(m_arena_block, locking::ekALL_HELD);
} /* do_visit() */

void nest_block_process::visit(crepr::base_block3D& block) {
  block.md()->reset_metrics();
  block.md()->distribution_time(mc_timestep);
} /* visit() */

NS_END(detail, operations, arena, cosm);
