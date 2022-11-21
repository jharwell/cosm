/**
 * \file block_extent_set.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/arena/operations/block_extent_set.hpp"

#include "cosm/arena/ds/arena_grid.hpp"
#include "cosm/ds/cell2D.hpp"
#include "cosm/ds/operations/cell2D_block_extent.hpp"
#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::operations::detail {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
block_extent_set::block_extent_set(crepr::sim_block3D* block)
    : ER_CLIENT_INIT("cosm.arena.operations.block_extent_set"), m_block(block) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_extent_set::visit(cads::arena_grid& grid) {
  auto xspan = m_block->xdspan();
  auto yspan = m_block->ydspan();

  /*
   * To reset all cells covered by the block's extent, we simply send them a
   * CELL_EMPTY event, EXCEPT for the cell that hosts the actual block, which is
   * not part of the extent.
   */
  for (size_t i = xspan.lb(); i <= xspan.ub(); ++i) {
    for (size_t j = yspan.lb(); j <= yspan.ub(); ++j) {
      rmath::vector2z c = rmath::vector2z(i, j);
      auto& cell = grid.access<cads::arena_grid::kCell>(i, j);
      if (c != m_block->danchor2D()) {
        ER_ASSERT(!cell.state_is_known() || cell.state_is_empty(),
                  "Cell@%s not unknown or empty [state=%d]",
                  rcppsw::to_string(c).c_str(),
                  cell.fsm().current_state());

        cdops::cell2D_block_extent_visitor e(c, m_block);
        e.visit(grid);
      }
    } /* for(j..) */
  } /* for(i..) */
} /* visit() */

} /* namespace cosm::arena::operations::detail */
