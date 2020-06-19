/**
 * \file free_block_pickup.cpp
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
#include "cosm/arena/operations/free_block_pickup.hpp"

#include "cosm/arena/base_arena_map.hpp"
#include "cosm/ds/operations/cell2D_empty.hpp"
#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, operations, detail);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
free_block_pickup::free_block_pickup(crepr::base_block3D* block,
                                     const rtypes::type_uuid& robot_id)
    : ER_CLIENT_INIT("cosm.arena.operations.free_block_pickup"),
      cell2D_op(block->dpos2D()),
      mc_robot_id(robot_id),
      m_block(block) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void free_block_pickup::visit(base_arena_map& map) {
  ER_ASSERT(m_block->dpos2D() == cell2D_op::coord(),
            "Coordinates for block/cell do not agree");
  RCSW_UNUSED auto old_r = m_block->rpos2D();

  cdops::cell2D_empty_visitor op(cell2D_op::coord());
  map.grid_mtx()->lock();
  op.visit(map.decoratee());
  map.grid_mtx()->unlock();

  ER_INFO("arena_map: fb%u: block%d@%s/%s",
          mc_robot_id.v(),
          m_block->id().v(),
          old_r.to_str().c_str(),
          cell2D_op::coord().to_str().c_str());
} /* visit() */

NS_END(detail, operations, arena, cosm);
