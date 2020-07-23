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
 * entity of \p ent1_dims dimensions anchored at \p ent1_anchor, (2) an entity
 * of \p ent1_dims that currently exists at \p ent1_anchor.
 */
placement_status_t placement_conflict2D(const rmath::vector2d& ent1_anchor,
                                        const rmath::vector2d& ent1_dims,
                                        const crepr::entity2D* entity);

/**
 * \brief Determine entity overlap with \p entity in two cases: (1) placing an
 * entity of \p ent1_dims dimensions anchored at \p ent1_loc, (2) an entity of
 * \p ent1_dims that currently exists at \p ent1_loc. Only 2D overlap is
 * considered (i.e. Z dimension is ignored).
 */
placement_status_t placement_conflict2D(const rmath::vector2d& ent1_anchor,
                                        const rmath::vector2d& ent1_dims,
                                        const crepr::entity3D* entity);

NS_END(utils, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_UTILS_UTILS_HPP_ */
