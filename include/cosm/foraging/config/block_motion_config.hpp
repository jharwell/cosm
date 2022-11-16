/**
 * \file block_motion_config.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "cosm/cosm.hpp"

#include "rcppsw/config/base_config.hpp"
#include "rcppsw/types/timestep.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct block_motion_config
 * \ingroup foraging config
 *
 * \brief Configuration for block motion of free blocks within the arena.
 */
struct block_motion_config final : public rconfig::base_config {
  double           random_walk_prob{0};
  std::string      policy{"Null"};
};

NS_END(config, foraging, cosm);

