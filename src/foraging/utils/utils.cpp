/**
 * \file loop_utils.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "cosm/foraging/utils/utils.hpp"
#include "cosm/repr/entity2D.hpp"
#include "cosm/arena/base_arena_map.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, utils);

/*******************************************************************************
 * Functions
 ******************************************************************************/
placement_status_t placement_conflict2D(const rmath::vector2d& ent1_loc,
                                      const rmath::vector2d& ent1_dims,
                                      const crepr::entity2D* const entity) {
  auto loc_xspan = crepr::entity2D::xspan(ent1_loc, ent1_dims.x());
  auto loc_yspan = crepr::entity2D::yspan(ent1_loc, ent1_dims.y());
  return placement_status_t{entity->xspan().overlaps_with(loc_xspan),
                            entity->yspan().overlaps_with(loc_yspan)};
} /* placement_conflict2D() */

placement_status_t placement_conflict2D(const rmath::vector2d& ent1_loc,
                                        const rmath::vector2d& ent1_dims,
                                        const crepr::entity3D* const entity) {
  auto loc_xspan = crepr::entity3D::xspan(ent1_loc, ent1_dims.x());
  auto loc_yspan = crepr::entity3D::yspan(ent1_loc, ent1_dims.y());
  return placement_status_t{entity->xspan().overlaps_with(loc_xspan),
                            entity->yspan().overlaps_with(loc_yspan)};
} /* placement_conflict2D() */

NS_END(utils, foraging, cosm);
