/**
 * \file cache_extent_clear.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/arena/operations/cache_extent_clear.hpp"

#include "cosm/arena/ds/arena_grid.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/ds/operations/cell2D_empty.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::operations::detail {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_extent_clear::cache_extent_clear(carepr::arena_cache* victim)
    : ER_CLIENT_INIT("cosm.arena.operations.cache_extent_clear"),
      m_victim(victim) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_extent_clear::visit(cads::arena_grid& grid) {
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
      auto& cell = grid.access<cads::arena_grid::kCell>(i, j);
      if (c != m_victim->dcenter2D()) {
        /*
         * These need to be warnings, not assert()s, because we can get here
         * when a dynamically created cache is found to be bad, which might
         * happen as a result of a bad center due to floating point
         * representation errors. In that case, we want to clear everything
         * out.
         */
        ER_CONDW(!cell.state_in_cache_extent(),
                 "Cell@%s not in CACHE_EXTENT [state=%d]--bad cache center?",
                 rcppsw::to_string(c).c_str(),
                 cell.fsm().current_state());

        ER_CONDW(!(cell.cache() == m_victim),
                 "Cache in cell@%s not victim cache%d--bad cache center",
                 rcppsw::to_string(c).c_str(),
                 m_victim->id().v());

        cdops::cell2D_empty_visitor e(c);
        e.visit(cell);
      }
    } /* for(j..) */
  } /* for(i..) */
} /* visit() */

} /* namespace cosm::arena::operations::detail */
