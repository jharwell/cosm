/**
 * \file actuation_subsystem2D_config.hpp
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
#include "cosm/kin2D/config/diff_drive_config.hpp"
#include "cosm/steer2D/config/force_calculator_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/

/**
 * \struct actuation_subsystem2D_config
 * \ingroup subsystem config
 *
 * \brief Configuration for the actuation subsystem for wheeled robots that
 * operate in two dimensions.
 */
struct actuation_subsystem2D_config final : public rconfig::base_config {
  ckin2D::config::diff_drive_config diff_drive{};
  csteer2D::config::force_calculator_config steering{};
};

NS_END(config, subsystem, cosm);

