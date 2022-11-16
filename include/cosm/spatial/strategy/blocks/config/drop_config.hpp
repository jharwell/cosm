/**
 * \file drop_config.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
  * Includes
******************************************************************************/
#include <string>

#include "rcppsw/config/base_config.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
  * Namespaces
******************************************************************************/
NS_START(cosm, spatial, strategy, blocks, config);

/*******************************************************************************
  * Structure Definitions
******************************************************************************/
/**
  * \struct drop_config
  * \ingroup spatial strategy blocks config
  *
  * \brief Configuration for block drop strategies that can be employed by
  * robots.
  */
struct drop_config final : public rconfig::base_config {
  /**
   * \brief The strategy to employ.
   */
  std::string strategy{};

  /**
   * \brief How long the strategy should be employed for (if relevant). E.g.,
   * how long to backup for.
   */
  rtypes::timestep duration{rtypes::constants::kNoTime};
};

NS_END(config, blocks, strategy, spatial, cosm);
