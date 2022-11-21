/**
 * \file block_manifest.hpp
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
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct block_manifest
 * \ingroup foraging config
 *
 * \brief Params of what types of blocks and how many of each should be
 * created, as well as size and other characteristics.
 */
struct block_manifest final : public rconfig::base_config {
  size_t n_cube{0};  /// # cube blocks to distribute in arena
  size_t n_ramp{0};  /// # ramp blocks to distribute in arena

  /**
   * \brief Size in meters of the unit dimension for blocks. Cube blocks are 1x1
   * in this dimension, and ramp blocks are 2x1.
   */
  double unit_dim{0.0};
};

} /* namespace cosm::foraging::config */

