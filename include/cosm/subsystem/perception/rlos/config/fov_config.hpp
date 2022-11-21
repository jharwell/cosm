/**
 * \file fov_config.hpp
 *
 * \copyright 2022 SIFT LLC, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "rcppsw/math/radians.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem::perception::rlos::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct fov_config
 * \ingroup subsystem perception rlos config
 *
 * \brief Configuration for robot reactive LOS perception sourced from a FOV.
 */
struct fov_config final : public rconfig::base_config {
  /**
   * \brief The maximum bearing angle between the agent's orientation and
   * something else to consider for inclusion in the FOV.
   */
  rmath::radians theta{};
};

} /* namespace cosm::subsystem::perception::rlos::config */
