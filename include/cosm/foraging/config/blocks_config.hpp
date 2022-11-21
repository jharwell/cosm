/**
 * \file blocks_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "cosm/foraging/config/block_dist_config.hpp"
#include "cosm/foraging/config/block_motion_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct blocks_config
 * \ingroup foraging config
 */
struct blocks_config final : public rconfig::base_config {
  block_dist_config dist{};
  block_motion_config motion{};
};

} /* namespace cosm::foraging::config */

