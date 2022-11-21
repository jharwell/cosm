/**
 * \file grid_config.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "rcppsw/spatial/euclidean_dist.hpp"

#include "cosm/ds/config/grid2D_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem::perception::rlos::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct grid_config
 * \ingroup subsystem perception rlos config
 *
 * \brief Configuration for robot reactive LOS perception sourced from a grid.
 */
struct grid_config final : public rconfig::base_config {
  rspatial::euclidean_dist dim{0};
  cds::config::grid2D_config grid2D {};
};

} /* namespace cosm::subsystem::perception::rlos::config */
