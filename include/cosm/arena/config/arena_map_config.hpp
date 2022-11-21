/**
 * \file arena_map_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/config/blocks_config.hpp"
#include "cosm/ds/config/grid2D_config.hpp"
#include "cosm/repr/config/nests_config.hpp"
#include "rcppsw/config/base_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct arena_map_config
 * \ingroup arena config
 */
struct arena_map_config final : public rconfig::base_config {
  struct cds::config::grid2D_config grid {};
  struct cfconfig::blocks_config blocks {};
  struct crepr::config::nests_config nests {};
};

} /* namespace cosm::arena::config */

