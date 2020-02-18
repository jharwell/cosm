/**
 * \file grid_config.hpp
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

#ifndef INCLUDE_COSM_DS_CONFIG_GRID_CONFIG_HPP_
#define INCLUDE_COSM_DS_CONFIG_GRID_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/types/discretize_ratio.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ds, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct grid_config
 * \ingroup config ds
 *
 * \brief Configuration for the \ref arena_grid used to represent the arena by
 * both loop functions and robots.
 */
struct grid_config final : public rconfig::base_config {
  rtypes::discretize_ratio resolution{0.0};
  rmath::vector2d upper{};
  rmath::vector2d lower{};
};

NS_END(config, ds, cosm);

#endif /* INCLUDE_COSM_DS_CONFIG_GRID_CONFIG_HPP_ */
