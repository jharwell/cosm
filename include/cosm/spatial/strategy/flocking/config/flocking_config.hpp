/**
 * \file flocking_config.hpp
 *
 * \copyright 2022 SIFT LLC, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
  * Includes
******************************************************************************/
#include <string>

#include "rcppsw/config/base_config.hpp"

#include "cosm/cosm.hpp"
#include "cosm/spatial/strategy/flocking/config/stoch_fov_config.hpp"

/*******************************************************************************
  * Namespaces
******************************************************************************/
namespace cosm::spatial::strategy::flocking::config {


/*******************************************************************************
  * Structure Definitions
******************************************************************************/
/**
  * \struct flocking_config
  * \ingroup spatial strategy flocking config
  *
  * \brief Configuration for flocking strategies that can be employed by agents.
  */
struct flocking_config final : public rconfig::base_config {
  /**
   * \brief The strategy to employ.
   */
  std::string strategy{"Null"};

  stoch_fov_config stoch_fov{};
};

} /* namespace cosm::spatial::strategy::flocking::config */
