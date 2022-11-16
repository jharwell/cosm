/**
 * \file polar_force_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/rcppsw.hpp"
#include "rcppsw/config/base_config.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct polar_force_config
 * \ingroup apf2D nav config
 *
 * \brief Configuration for the \ref polar_force.
 */
struct polar_force_config final : public rconfig::base_config {
  /**
   * The upper limit for polar force magnitude.
   */
  double max{0};
};

} /* namespace cosm::apf2D::nav::config */
