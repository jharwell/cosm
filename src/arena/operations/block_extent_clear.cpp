/**
 * \file block_extent_clear.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/arena/operations/block_extent_clear.hpp"

#include "cosm/arena/ds/arena_grid.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/ds/operations/cell2D_empty.hpp"
#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, operations, detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_extent_clear::block_extent_clear(crepr::sim_block3D* victim)
    : ER_CLIENT_INIT("cosm.arena.operations.block_extent_clear"),
      m_victim(victim) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_extent_clear::visit(cads::arena_grid& grid) {
  auto xspan = m_victim->xdspan();
  auto yspan = m_victim->ydspan();

  /*
   * To reset all cells covered by the block's extent, we simply send them a
   * CELL_EMPTY event, EXCEPT for the cell that hosts the actual block, which is
   * not part of the extent.
   */
  for (size_t i = xspan.lb(); i <= xspan.ub(); ++i) {
    for (size_t j = yspan.lb(); j <= yspan.ub(); ++j) {
      rmath::vector2z c = rmath::vector2z(i, j);
      auto& cell = grid.access<cads::arena_grid::kCell>(i, j);
      if (c != m_victim->danchor2D()) {
        ER_ASSERT(cell.state_in_block_extent(),
                  "Cell@%s not in BLOCK_EXTENT [state=%d]",
                  rcppsw::to_string(c).c_str(),
                  cell.fsm().current_state());

        ER_ASSERT(cell.block3D() == m_victim,
                  "Block in cell@%s not victim block%d",
                  rcppsw::to_string(c).c_str(),
                  m_victim->id().v());

        cdops::cell2D_empty_visitor e(c);
        e.visit(cell);
      }
    } /* for(j..) */
  } /* for(i..) */
} /* visit() */

NS_END(detail, operations, arena, cosm);
