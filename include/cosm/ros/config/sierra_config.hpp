/**
 * \file sierra_config.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "cosm/cosm.hpp"

#include "rcppsw/config/base_config.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/types/hertz.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ros::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
struct sierra_experiment_config {
  rtypes::timestep length{rtypes::constants::kNoTime};
  std::string param_file{};
  size_t n_robots{0};
  rtypes::hertz ticks_per_sec{rtypes::constants::kNoRate};
  bool barrier_start{false};
};

/**
 * \struct sierra_config
 * \ingroup ros config
 *
 * \brief Configuration for parameters set by SIERRA.
 */
struct sierra_config final : public rconfig::base_config {
  sierra_experiment_config experiment{};
};

} /* namespace cosm::ros::config */
