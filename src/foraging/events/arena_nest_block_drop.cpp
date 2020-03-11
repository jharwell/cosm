/**
 * \file arena_nest_block_drop.cpp
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
#include "cosm/foraging/events/arena_nest_block_drop.hpp"

#include "cosm/foraging/ds/arena_map.hpp"
#include "cosm/repr/base_block2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, events, detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
arena_nest_block_drop::arena_nest_block_drop(
    std::unique_ptr<crepr::base_block2D> robot_block,
    const rtypes::timestep& t)
    : ER_CLIENT_INIT("cosm.events.arena_nest_block_drop"),
      mc_timestep(t),
      m_robot_block(std::move(robot_block)) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void arena_nest_block_drop::visit(cfds::arena_map& map) {
  ER_ASSERT(rtypes::constants::kNoUUID != m_robot_block->md()->robot_id(),
            "Undefined robot index");

  /*
   * We don't need the cache mutex held here, BUT we do need the other two, and
   * all mutexes always have to be acquired in the same order everywhere in
   * order to avoid deadlocks. If we let arena map acquire the cache mutex
   * during block distribution, we can get a deadlock due to ordering.
   */
  std::scoped_lock lock1(*map.cache_mtx());
  std::scoped_lock lock2(*map.block_mtx());
  std::scoped_lock lock3(*map.grid_mtx());

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
                              cfds::arena_map_locking::ekALL_HELD);
  visit(*m_arena_block);
} /* visit() */

void arena_nest_block_drop::visit(crepr::base_block2D& block) {
  block.md()->reset_metrics();
  block.md()->distribution_time(mc_timestep);
} /* visit() */

NS_END(detail, events, foraging, cosm);
