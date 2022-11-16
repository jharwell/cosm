/**
 * \file acq_config.hpp
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
NS_START(cosm, spatial, strategy, nest, config);

/*******************************************************************************
  * Structure Definitions
******************************************************************************/
/**
  * \struct acq_config
  * \ingroup spatial strategy nest config
  *
  * \brief Configuration for nest acquisition strategies that can be employed by
  * robots.
  */
struct acq_config final : public rconfig::base_config {
  /**
   * \brief The strategy to employ.
   */
  std::string strategy{};

  /**
   * \brief How long the strategy should be employed for (if relevant). E.g.,
   * how long to wander for.
   */
  rtypes::timestep duration{rtypes::constants::kNoTime};
};

NS_END(config, nest, strategy, spatial, cosm);
