/**
 * \file perception_config.hpp
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

#ifndef INCLUDE_COSM_SUBSYSTEM_CONFIG_PERCEPTION_PERCEPTION_CONFIG_HPP_
#define INCLUDE_COSM_SUBSYSTEM_CONFIG_PERCEPTION_PERCEPTION_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"

#include "cosm/ds/config/grid2D_config.hpp"
#include "cosm/subsystem/perception/config/pheromone_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem, perception, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct perception_config
 * \ingroup subsystem perception config
 *
 * \brief Configuration for robot perception.
 */
struct perception_config final : public rconfig::base_config {
  double los_dim{-1};
  cds::config::grid2D_config occupancy_grid {};
  struct pheromone_config pheromone {};
};

NS_END(config, perception, subsystem, cosm);

#endif /* INCLUDE_COSM_SUBSYSTEM_PERCEPTION_CONFIG_PERCEPTION_CONFIG_HPP_ */
