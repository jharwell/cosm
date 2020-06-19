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
 * Forward Declarations
 ******************************************************************************/
static void do_lock(caching_arena_map& map);
static void do_lock(base_arena_map& map);
static void do_unlock(caching_arena_map& map);
static void do_unlock(base_arena_map& map);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
nest_block_process::nest_block_process(
    std::unique_ptr<crepr::base_block3D> robot_block,
    const rtypes::timestep& t)
    : ER_CLIENT_INIT("cosm.arena.operations.nest_block_process"),
      mc_timestep(t),
      mc_robot_block_id(robot_block->id()) {}

nest_block_process::nest_block_process(const rtypes::type_uuid& robot_block_id,
                                       const rtypes::timestep& t)
    : ER_CLIENT_INIT("cosm.arena.operations.nest_block_process"),
      mc_timestep(t),
      mc_robot_block_id(robot_block_id) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void nest_block_process::visit(base_arena_map& map) {
  do_lock(map);
  do_visit(map);
  do_unlock(map);
} /* visit() */

void nest_block_process::visit(caching_arena_map& map) {
  do_lock(map);
  do_visit(map);
  do_unlock(map);
} /* visit() */

template <typename TArenaMap>
void nest_block_process::do_visit(TArenaMap& map) {
  /*
   * The robot owns a unique copy of a block originally from the arena, so we
   * need to look it up rather than implicitly converting its unique_ptr to a
   * shared_ptr and distributing it--this will cause lots of problems later.
   */
  auto it =
      std::find_if(map.blocks().begin(),
                   map.blocks().end(),
                   [&](const auto& b) { return mc_robot_block_id == b->id(); });
  ER_ASSERT(map.blocks().end() != it,
            "Robot block%s not found in arena map blocks",
            rcppsw::to_string(mc_robot_block_id).c_str());
  m_arena_block = *it;

  /* update block after processing */
  visit(*m_arena_block);

  /* release block back into the wild */
  map.distribute_single_block(m_arena_block, arena_map_locking::ekALL_HELD);
} /* do_visit() */

void nest_block_process::visit(crepr::base_block3D& block) {
  block.md()->reset_metrics();
  block.md()->distribution_time(mc_timestep);
} /* visit() */

/*******************************************************************************
 * Free Functions
 ******************************************************************************/
void do_lock(caching_arena_map& map) {
  /*
   * We don't need the cache mutex held here, BUT we do need the other two, and
   * all mutexes always have to be acquired in the same order everywhere in
   * order to avoid deadlocks. If we let arena map acquire the cache mutex
   * during block distribution, we can get a deadlock due to ordering.
   */
  map.cache_mtx()->lock();
  map.block_mtx()->lock();
  map.grid_mtx()->lock();
} /* do_lock() */

void do_unlock(caching_arena_map& map) {
  map.grid_mtx()->unlock();
  map.block_mtx()->unlock();
  map.cache_mtx()->unlock();
} /* do_unlock() */

void do_lock(base_arena_map& map) {
  map.block_mtx()->lock();
  map.grid_mtx()->lock();
} /* do_lock() */

void do_unlock(base_arena_map& map) {
  map.grid_mtx()->unlock();
  map.block_mtx()->unlock();
} /* do_unlock() */

NS_END(detail, operations, arena, cosm);
