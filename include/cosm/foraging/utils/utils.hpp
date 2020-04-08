/**
 * \file loop_utils.hpp
 * \ingroup foraging utils
 *
 * Helpers for loop functions that CAN be free functions, as they do not require
 * access to anything in \ref argos::CLoopFunctions.
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

#ifndef INCLUDE_COSM_FORAGING_UTILS_UTILS_HPP_
#define INCLUDE_COSM_FORAGING_UTILS_UTILS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/math/vector2.hpp"
#include "rcppsw/math/vector3.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {
class entity2D;
class entity3D;
} /* namespace cosm::repr */

namespace cosm::arena {
template<typename T>
class base_arena_map;
} /* namespace cosm::foraging::repr */

namespace cosm::foraging::repr {
class foraging_los;
}

NS_START(cosm, foraging, utils);

/*******************************************************************************
 * Types
 ******************************************************************************/
struct placement_status_t {
  bool x_conflict{false};
  bool y_conflict{false};
};

/*******************************************************************************
 * Functions
 ******************************************************************************/
/**
 * \brief Determine entity overlap with \p entity in two cases: (1) placing an
 * entity of \p ent1_dims dimensions placed at \p ent1_loc, (2) an entity of \p
 * ent1_dims that currently exists at \p ent1_loc.
 */
placement_status_t placement_conflict2D(const rmath::vector2d& ent1_loc,
                                        const rmath::vector2d& ent1_dims,
                                        const crepr::entity2D* entity);

/**
 * \brief Determine entity overlap with \p entity in two cases: (1) placing an
 * entity of \p ent1_dims dimensions placed at \p ent1_loc, (2) an entity of \p
 * ent1_dims that currently exists at \p ent1_loc. Only 2D overlap is considered
 * (i.e. Z dimension is ignored).
 */
placement_status_t placement_conflict2D(const rmath::vector2d& ent1_loc,
                                        const rmath::vector2d& ent1_dims,
                                        const crepr::entity3D* entity);

/**
 * \brief Compute the line of sight for a given robot.
 *
 * Needed to eliminate header dependencies in this file.
 */
template<typename TArenaMapType>
std::unique_ptr<cfrepr::foraging_los> compute_robot_los(
    const TArenaMapType& map,
    uint los_grid_size,
    const rmath::vector2d& pos) {
  rmath::vector2u position = rmath::dvec2uvec(pos, map.grid_resolution().v());
  return std::make_unique<cfrepr::foraging_los>(
      map.subgrid(position.x(), position.y(), los_grid_size), position);
} /* compute_robot_los */

/**
 * \brief Set the LOS of a robot in the arena.
 *
 * This is a hack that makes getting my research up and running easier.
 *
 * \todo This should eventually be replaced by a calculation of a robot's LOS by
 * the robot, probably using on-board cameras.
 */
template <typename TControllerType, typename TBlockType>
void set_robot_los(TControllerType* const controller,
                   uint los_grid_size,
                   carena::base_arena_map<TBlockType>& map) {
  controller->los(std::move(compute_robot_los(map,
                                              los_grid_size,
                                              controller->pos2D())));
}

NS_END(utils, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_UTILS_UTILS_HPP_ */
