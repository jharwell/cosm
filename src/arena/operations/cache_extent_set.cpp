/**
 * \file cache_extent_set.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/arena/operations/cache_extent_set.hpp"

#include "cosm/arena/caching_arena_map.hpp"
#include "cosm/arena/repr/arena_cache.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/ds/operations/cell2D_cache_extent.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, operations, detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
cache_extent_set::cache_extent_set(carepr::arena_cache* cache)
    : ER_CLIENT_INIT("cosm.arena.operations.cache_extent_set"), m_cache(cache) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void cache_extent_set::visit(cads::arena_grid& grid) {
  auto xspan = m_cache->xdspan();
  auto yspan = m_cache->ydspan();

  for (size_t i = xspan.lb(); i <= xspan.ub(); ++i) {
    for (size_t j = yspan.lb(); j <= yspan.ub(); ++j) {
      auto dcoord = rmath::vector2z(i, j);
      RCPPSW_UNUSED auto rcoord = rmath::zvec2dvec(dcoord,
                                                   grid.resolution().v());
      auto& cell = grid.access<cads::arena_grid::kCell>(i, j);

      ER_CONDW(!m_cache->contains_point(rcoord),
               "Cache%d@%s/%s xspan=%s,yspan=%s does not contain %s",
               m_cache->id().v(),
               rcppsw::to_string(m_cache->rcenter2D()).c_str(),
               rcppsw::to_string(m_cache->dcenter2D()).c_str(),
               rcppsw::to_string(m_cache->xrspan()).c_str(),
               rcppsw::to_string(m_cache->yrspan()).c_str(),
               rcppsw::to_string(rcoord).c_str());

      if (dcoord != m_cache->dcenter2D()) {
        ER_ASSERT(!cell.state_is_known() || cell.state_is_empty(),
                  "Cell@%s not unknown or empty [state=%d]",
                  rcppsw::to_string(dcoord).c_str(),
                  cell.fsm().current_state());

        cdops::cell2D_cache_extent_visitor e(dcoord, m_cache);
        e.visit(grid);
      }
    } /* for(j..) */
  } /* for(i..) */
} /* visit() */

NS_END(detail, operations, arena, cosm);
