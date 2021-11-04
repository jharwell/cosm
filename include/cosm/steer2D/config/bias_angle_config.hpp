/**
 * \file bias_angle_config.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_STEER2D_CONFIG_BIAS_ANGLE_CONFIG_HPP_
#define INCLUDE_COSM_STEER2D_CONFIG_BIAS_ANGLE_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <string>

#include "rcppsw/config/base_config.hpp"
#include "rcppsw/math/radians.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct bias_angle_config
 * \ingroup steer2D config
 *
 * \brief Configuration for angle generation as part of the \ref wander_force.
 */
struct bias_angle_config final : public rconfig::base_config {
  /**
   * \brief The bias angle source.
   */
  std::string src{};

  /**.
   * \brief Defines the range [0, value] from which a random value will be
   * sampled uniformly each timestep in order to determine how much +/- angle
   * deviation from a straight line trajectory should be applied. High value =
   * higher average per-timestep deviation/more visibly random wandering.
   */
  rmath::radians max_delta{-1};

  /**
   * \brief Defines the set of specific angle values to choose between when
   * selected the bias angle each timestep.
   */
  std::vector<rmath::radians> angles{};
};

NS_END(config, steer2D, cosm);

#endif /* INCLUDE_COSM_STEER2D_CONFIG_BIAS_ANGLE_CONFIG_HPP_ */
