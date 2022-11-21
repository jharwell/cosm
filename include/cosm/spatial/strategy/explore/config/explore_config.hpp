/**
 * \file explore_config.hpp
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
namespace cosm::spatial::strategy::explore::config {

/*******************************************************************************
  * Structure Definitions
******************************************************************************/
/**
  * \struct explore_config
  * \ingroup spatial strategy explore config
  *
  * \brief Configuration for exploration strategies that can be employed by
  * robots.
  */
struct explore_config final : public rconfig::base_config {
  /**
   * \brief The strategy to employ.
   */
  std::string strategy{};

  /**
   * \brief The minimum amount of time the strategy must be executed for.
   *
   * This is helpful in simulation to:
   *
   * - Ensure that robots don't think they have acquired a block on the first
   *   timestep (for example), before everything has fully initialized.
   *
   * - Force some tasks to not pick up the block they just dropped if it is the
   *   only one they know about (The exceptions list disables vectoring to it,
   *   BUT they can still explore for it, and without this minimum they will
   *   immediately acquire it and bypass the list).
   */
  rtypes::timestep min_duration{rtypes::constants::kNoTime};
};

} /* namespace cosm::spatial::strategy::explore::config */
