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

#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/ds/arena_grid.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/ds/operations/cell2D_empty.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, operations, detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_extent_clear::cache_extent_clear(carepr::arena_cache* victim)
    : ER_CLIENT_INIT("cosm.arena.operations.cache_extent_clear"),
      m_victim(victim) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_extent_clear::visit(cds::arena_grid& grid) {
  auto xspan = m_victim->xdspan();
  auto yspan = m_victim->ydspan();

  /*
   * To reset all cells covered by the cache's extent, we simply send them a
   * CELL_EMPTY event. EXCEPT for the cell that hosted the actual cache, because
   * it is currently in the HAS_BLOCK state as part of a \ref cached_block_pickup,
   * and clearing it here will trigger an assert later.
   */
  for (size_t i = xspan.lb(); i <= xspan.ub(); ++i) {
    for (size_t j = yspan.lb(); j <= yspan.ub(); ++j) {
      rmath::vector2z c = rmath::vector2z(i, j);
      auto& cell = grid.access<cds::arena_grid::kCell>(i, j);
      if (c != m_victim->dcenter2D()) {
        ER_ASSERT(cell.state_in_cache_extent(),
                  "Cell@%s not in CACHE_EXTENT [state=%d]",
                  rcppsw::to_string(c).c_str(),
                  cell.fsm().current_state());

        ER_ASSERT(cell.cache() == m_victim,
                  "Cache in cell@%s not victim cache%d",
                  rcppsw::to_string(c).c_str(),
                  m_victim->id().v());

        cdops::cell2D_empty_visitor e(c);
        e.visit(cell);
      }
    } /* for(j..) */
  } /* for(i..) */
} /* visit() */

NS_END(detail, operations, arena, cosm);
