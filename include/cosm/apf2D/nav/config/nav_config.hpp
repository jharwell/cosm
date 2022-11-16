/**
 * \file nav_config.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/apf2D/nav/config/avoidance_force_config.hpp"
#include "cosm/apf2D/nav/config/arrival_force_config.hpp"
#include "cosm/apf2D/nav/config/wander_force_config.hpp"
#include "cosm/apf2D/nav/config/polar_force_config.hpp"
#include "cosm/apf2D/nav/config/phototaxis_force_config.hpp"
#include "cosm/apf2D/nav/config/path_following_force_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct nav_config
 * \ingroup apf2D nav config
 *
 * \brief Configuration force APF forces related to navigation (i.e., forces
 * that agents use to GO places in some manner).
 */
struct nav_config : public rconfig::base_config {
  avoidance_force_config      avoidance{};
  arrival_force_config        arrival{};
  wander_force_config         wander{};
  polar_force_config          polar{};
  phototaxis_force_config     phototaxis{};
  path_following_force_config path_following{};
};

} /* namespace cosm::apf2D::nav::config */
