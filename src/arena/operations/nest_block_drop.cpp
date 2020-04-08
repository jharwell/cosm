/**
 * \file nest_block_drop.cpp
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
#include "cosm/arena/operations/nest_block_drop.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/repr/base_block2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, operations, detail);

/*******************************************************************************
 * Forward Declarations
 ******************************************************************************/
static void do_lock(caching_arena_map& map);
static void do_lock(base_arena_map<crepr::base_block2D>& map);
static void do_unlock(caching_arena_map& map);
static void do_unlock(base_arena_map<crepr::base_block2D>& map);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
nest_block_drop::nest_block_drop(
    std::unique_ptr<crepr::base_block2D> robot_block,
    const rtypes::timestep& t)
    : ER_CLIENT_INIT("cosm.operations.nest_block_drop"),
      mc_timestep(t),
      m_robot_block(std::move(robot_block)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void nest_block_drop::visit(base_arena_map<crepr::base_block2D>& map) {
  ER_ASSERT(rtypes::constants::kNoUUID != m_robot_block->md()->robot_id(),
            "Undefined robot index");

  do_lock(map);
  do_visit(map);
  do_unlock(map);
} /* visit() */

void nest_block_drop::visit(caching_arena_map& map) {
  ER_ASSERT(rtypes::constants::kNoUUID != m_robot_block->md()->robot_id(),
            "Undefined robot index");

  do_lock(map);
  do_visit(map);
  do_unlock(map);
} /* visit() */

template <typename TArenaMapType>
void nest_block_drop::do_visit(TArenaMapType& map) {
  /*
   * The robot owns a unique copy of a block originally from the arena, so we
   * need to look it up rather than implicitly converting its unique_ptr to a
   * shared_ptr and distributing it--this will cause lots of problems later.
   */
  auto it =
      std::find_if(map.blocks().begin(), map.blocks().end(), [&](const auto& b) {
          return m_robot_block->id() == b->id();
        });
  ER_ASSERT(map.blocks().end() != it,
            "Robot block%d not found in arena map blocks",
            m_robot_block->id().v());
  m_arena_block = *it;
  map.distribute_single_block(m_arena_block,
                              arena_map_locking::ekALL_HELD);
} /* do_visit() */

void nest_block_drop::visit(crepr::base_block2D& block) {
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
  map.grid_mtx()->lock();
  map.block_mtx()->lock();
  map.cache_mtx()->lock();
} /* do_unlock() */

void do_lock(base_arena_map<crepr::base_block2D>& map) {
  map.block_mtx()->lock();
  map.grid_mtx()->lock();
} /* do_lock() */

void do_unlock(base_arena_map<crepr::base_block2D>& map) {
  map.grid_mtx()->lock();
  map.block_mtx()->lock();
} /* do_unlock() */

NS_END(detail, operations, arena, cosm);
