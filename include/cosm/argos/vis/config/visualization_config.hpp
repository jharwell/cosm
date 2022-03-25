/**
 * \file visualization_config.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, argos, vis, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct visualization_config
 * \ingroup vis config
 *
 * \brief Configuration for extended ARGoS visualizations.
 */
struct visualization_config : public rconfig::base_config {
  bool robot_id{false};
  bool robot_los{false};
  bool robot_task{false};
  bool robot_steer2D{false};
  bool block_id{false};
};

NS_END(config, vis, argos, cosm);
