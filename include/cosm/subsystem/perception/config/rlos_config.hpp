/**
 * \file rlos_config.hpp
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

#include "cosm/ds/config/grid2D_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem, perception, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct rlos_config
 * \ingroup subsystem perception config
 *
 * \brief Configuration for robot reactive LOS perception.
 */
struct rlos_config final : public rconfig::base_config {
  double dim{-1};
  cds::config::grid2D_config grid2D {};
};

NS_END(config, perception, subsystem, cosm);

