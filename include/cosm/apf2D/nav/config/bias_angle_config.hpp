/**
 * \file bias_angle_config.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

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
namespace cosm::apf2D::nav::config {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct bias_angle_config
 * \ingroup apf2D nav config
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

} /* namespace cosm::apf2D::nav::config */
