/**
 * \file output_config.hpp
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

#ifndef INCLUDE_COSM_PAL_CONFIG_OUTPUT_CONFIG_HPP_
#define INCLUDE_COSM_PAL_CONFIG_OUTPUT_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/config/base_config.hpp"

#include "rcppsw/metrics/config/metrics_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, pal, config);

namespace fs = std::filesystem;

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct output_config
 * \ingroup pal config
 *
 * \brief Configuration for robot/swarm manager output.
 */
struct output_config final : public rconfig::base_config {
  /**
   * \brief Absolute or relative path to the parent directory of the output root
   *        for the swarm manager/robot.
   */
  fs::path                 output_parent{};

  /**
   * \brief Directory name within the output parent that things should be output
   * into. This is a separate argument than output_root, because there are
   * special values of it that have different behavior.
   *
   * __current_date__ will cause \p output_leaf to be set to the current date in
   * the format "Y-M-D:H-M".
   */
  std::string              output_leaf{};

  rmconfig::metrics_config metrics {};

  static fs::path root_calc(const output_config* const config);
};

NS_END(config, pal, cosm);

#endif /* INCLUDE_COSM_PAL_CONFIG_OUTPUT_CONFIG_HPP_ */
