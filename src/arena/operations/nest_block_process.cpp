/**
 * \file nest_block_process.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/arena/operations/nest_block_process.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, operations, detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
nest_block_process::nest_block_process(crepr::sim_block3D* arena_block,
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

void nest_block_process::visit(crepr::sim_block3D& block) {
  block.md()->reset_metrics();
  block.md()->distribution_time(mc_timestep);
} /* visit() */

NS_END(detail, operations, arena, cosm);
