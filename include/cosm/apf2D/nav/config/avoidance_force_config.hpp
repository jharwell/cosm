/**
 * \file avoidance_force_config.hpp
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
 * Struct Definitions
 ******************************************************************************/
/**
 * \struct avoidance_force_config
 * \ingroup apf2D nav config
 *
 * \brief Configuration for the virtual avoidance force, as described in \todo
 * ref.
 */
struct avoidance_force_config final : public rconfig::base_config {
  /**
   * The upper limit for avoidance force magnitude.
   */
  double max{0};
};

} /* namespace cosm::apf2D::nav::config */
