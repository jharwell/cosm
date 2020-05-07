/**
 * \file penalty_id_calculator.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
#include "cosm/foraging/tv/penalty_id_calculator.hpp"

#include "cosm/arena/base_arena_map.hpp"
#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, foraging, tv);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rtypes::type_uuid penalty_id_calculator::from_nest_drop(
    const crepr::base_block3D* block) const {
  ER_ASSERT(nullptr != block &&
            rtypes::constants::kNoUUID != block->id(),
            "Robot not carrying block?");
  return block->id();
} /* from_nest_drop() */

rtypes::type_uuid penalty_id_calculator::from_free_pickup(
    const rmath::vector2d& loc,
    const rtypes::type_uuid& acq_id,
    const carena::base_arena_map* map) const {
  auto id = map->robot_on_block(loc, acq_id);
  ER_ASSERT(rtypes::constants::kNoUUID != id, "Robot not on block?");
  return id;
} /* from_free_pickup() */


NS_END(tv, foraging, cosm);
