/**
 * \file base_env_dynamics_config.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"

#include "cosm/tv/config/temporal_penalty_config.hpp"
#include "cosm/tv/config/robot_dynamics_applicator_config.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::tv::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct base_env_dynamics_config
 * \ingroup tv config
 *
 * \brief Base configuration that can be reused for the environmental dynamics
 * managers defined in projects built on COSM.
 */
struct base_env_dynamics_config : public rconfig::base_config {
  ctv::config::robot_dynamics_applicator_config rda{};
  ctv::config::temporal_penalty_config block_manip_penalty{};
};

} /* namespace cosm::config::tv */

