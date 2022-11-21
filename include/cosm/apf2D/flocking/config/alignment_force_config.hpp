/**
 * \file alignment_force_config.hpp
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

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::flocking::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct alignment_force_config
 * \ingroup apf2D flocking config
 *
 * \brief Configuration for \ref capf2D::flocking::alignment_force.
 */
struct alignment_force_config : public rconfig::base_config {
  /**
   * \brief The maximum strength of the force.
   */
  double max{0};
};

} /* namespace cosm::apf2D::flocking::config */
