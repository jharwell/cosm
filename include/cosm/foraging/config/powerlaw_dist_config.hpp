/**
 * \file powerlaw_dist_config.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
 * \struct powerlaw_dist_config
 * \ingroup foraging config
 *
 * \brief Configuration for powerlaw block distribution.
 */
struct powerlaw_dist_config final : public rconfig::base_config {
  /**
   * \brief Min power of 2 for distribution.
   */
  size_t pwr_min{0};

  /**
   * \brief Max power of 2 for distribution.
   */
  size_t pwr_max{0};

  /**
   * \brief How many clusters to allocate in the arena.
   */
  size_t n_clusters{0};
};

} /* namespace cosm::foraging::config */
