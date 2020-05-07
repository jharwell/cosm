/**
 * \file cache_extent_clear.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "cosm/arena/operations/cache_extent_clear.hpp"

#include "cosm/ds/cell2D.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/ds/operations/cell2D_empty.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, operations, detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_extent_clear::cache_extent_clear(const rmath::vector2z& coord,
                                       carepr::arena_cache* victim)
    : ER_CLIENT_INIT("cosm.arena.operations.cache_extent_clear"),
      cell2D_op(coord),
      m_victim(victim) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_extent_clear::visit(caching_arena_map& map) {
  visit(map.decoratee());
} /* visit() */

void cache_extent_clear::visit(cds::arena_grid& grid) {
  auto xspan = m_victim->xspan();
  auto yspan = m_victim->yspan();

  /*
   * To reset all cells covered by the cache's extent, we simply send them a
   * CELL_EMPTY event. EXCEPT for the cell that hosted the actual cache, because
   * it is currently in the HAS_BLOCK state as part of a \ref cached_block_pickup,
   * and clearing it here will trigger an assert later.
   */
  auto xmin = static_cast<uint>(std::ceil(xspan.lb() / grid.resolution().v()));
  auto xmax = static_cast<uint>(std::ceil(xspan.ub() / grid.resolution().v()));
  auto ymin = static_cast<uint>(std::ceil(yspan.lb() / grid.resolution().v()));
  auto ymax = static_cast<uint>(std::ceil(yspan.ub() / grid.resolution().v()));

  for (uint i = xmin; i < xmax; ++i) {
    for (uint j = ymin; j < ymax; ++j) {
      rmath::vector2z c = rmath::vector2z(i, j);
      if (c != m_victim->dpos2D()) {
        ER_ASSERT(m_victim->contains_point2D(
            rmath::zvec2dvec(c, grid.resolution().v())),
                  "Cache%d does not contain point (%u, %u) within its extent",
                  m_victim->id().v(),
                  i,
                  j);

        auto& cell = grid.access<cds::arena_grid::kCell>(i, j);
        ER_ASSERT(cell.state_in_cache_extent(),
                  "cell(%u, %u) not in CACHE_EXTENT [state=%d]",
                  i,
                  j,
                  cell.fsm().current_state());
        cdops::cell2D_empty_visitor e(c);
        e.visit(cell);
      }
    } /* for(j..) */
  }   /* for(i..) */
} /* visit() */

NS_END(detail, operations, arena, cosm);
