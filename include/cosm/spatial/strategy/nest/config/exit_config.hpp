/**
 * \file exit_config.hpp
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
namespace cosm::spatial::strategy::nest::config {

/*******************************************************************************
  * Structure Definitions
******************************************************************************/
/**
  * \struct exit_config
  * \ingroup spatial strategy nest config
  *
  * \brief Configuration for nest exit strategies that can be employed by
  * robots.
  */
struct exit_config final : public rconfig::base_config {
  /**
   * \brief The strategy to employ.
   */
  std::string strategy{};
};

} /* namespace cosm::spatial::strategy::nest::config */
