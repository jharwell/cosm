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

#include "cosm/subsystem/perception/rlos/config/grid_config.hpp"
#include "cosm/subsystem/perception/rlos/config/fov_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem::perception::rlos::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct rlos_config
 * \ingroup subsystem perception rlos config
 *
 * \brief Configuration for robot reactive LOS perception.
 */
struct rlos_config final : public rconfig::base_config {
  /* clang-format off */
  grid_config grid{};
  fov_config  fov{};
  /* clang-format on */
};

} /* namespace cosm::subsystem::perception::rlos::config */
