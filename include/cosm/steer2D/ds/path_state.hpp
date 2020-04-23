/**
 * \file path_state.hpp
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

#ifndef INCLUDE_COSM_STEER2D_DS_PATH_STATE_HPP_
#define INCLUDE_COSM_STEER2D_DS_PATH_STATE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/math/vector2.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D, ds);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \class path_state
 * \ingroup steer2D ds
 *
 * \brief Holds the overall path for \ref path_following_force, as well as the
 * robot's current progress along it.
 */
struct path_state {
    /* clang-format off */
  std::vector<rmath::vector2d> path{};
  size_t                       current_node{0};
  /* clang-format on */
};

NS_END(ds, steer2D, cosm);

#endif /* INCLUDE_STEER2D_DS_COSM_PATH_STATE_HPP_ */
