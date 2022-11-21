/**
 * \file apf_manager_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/apf2D/nav/config/nav_config.hpp"
#include "cosm/apf2D/flocking/config/flocking_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct apf_manager_config
 * \ingroup apf2D config
 */
struct apf_manager_config final : public rconfig::base_config {
  /* clang-format off */
  nav::config::nav_config nav{};
  flocking::config::flocking_config flocking{};
  /* clang-format on */
};

} /* namespace cosm::apf2D::config */
