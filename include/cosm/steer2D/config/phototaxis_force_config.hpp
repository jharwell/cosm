/**
 * \file phototaxis_force_config.hpp
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
NS_START(cosm, steer2D, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct phototaxis_force_config
 * \ingroup steer2D config
 *
 * \brief Configuration for virtual phototaxis force.
 */
struct phototaxis_force_config final : public rconfig::base_config {
  double max{0};
};

NS_END(config, steer2D, cosm);

