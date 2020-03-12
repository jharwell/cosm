/**
 * \file metrics_config.hpp
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

#ifndef INCLUDE_COSM_METRICS_CONFIG_METRICS_CONFIG_HPP_
#define INCLUDE_COSM_METRICS_CONFIG_METRICS_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <map>
#include <string>

#include "rcppsw/config/base_config.hpp"
#include "rcppsw/rcppsw.hpp"
#include "rcppsw/types/timestep.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, metrics, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
struct metrics_output_mode_config {
  using enabled_map_type = std::map<std::string, std::string>;
  rtypes::timestep output_interval{0};
  enabled_map_type enabled{};
};

/**
 * \struct metrics_config
 * \ingroup metrics config
 */
struct metrics_config final : public rconfig::base_config {
  std::string                output_dir{};
  metrics_output_mode_config append{};
  metrics_output_mode_config truncate{};
  metrics_output_mode_config create{};
};

NS_END(config, metrics, cosm);

#endif /* INCLUDE_COSM_METRICS_CONFIG_METRICS_CONFIG_HPP_ */
