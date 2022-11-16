/**
 * \file polar_force_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/rcppsw.hpp"
#include "rcppsw/config/base_config.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D, config);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \struct polar_force_config
 * \ingroup steer2D config
 *
 * \brief Configuration for the polar virtual force.
 */
struct polar_force_config final : public rconfig::base_config {
  /**
   * The upper limit for polar force magnitude.
   */
  double max{0};
};

NS_END(config, steer2D, cosm);

