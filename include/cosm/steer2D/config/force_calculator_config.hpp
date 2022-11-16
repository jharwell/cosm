/**
 * \file force_calculator_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/rcppsw.hpp"
#include "cosm/steer2D/config/avoidance_force_config.hpp"
#include "cosm/steer2D/config/arrival_force_config.hpp"
#include "cosm/steer2D/config/wander_force_config.hpp"
#include "cosm/steer2D/config/polar_force_config.hpp"
#include "cosm/steer2D/config/phototaxis_force_config.hpp"
#include "cosm/steer2D/config/path_following_force_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \struct force_calculator_config
 * \ingroup steer2D config
 */
struct force_calculator_config final : public rconfig::base_config {
  /* clang-format off */
  avoidance_force_config      avoidance{};
  arrival_force_config        arrival{};
  wander_force_config         wander{};
  polar_force_config          polar{};
  phototaxis_force_config     phototaxis{};
  path_following_force_config path_following{};
  /* clang-format on */
};

NS_END(config, steer2D, cosm);

