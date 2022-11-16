/**
 * \file path_following_force_config.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
NS_START(cosm, steer2D, config);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * \struct path_following_force_config
 * \ingroup steer2D config
 *
 * \brief Configuration for the virtual path following force, as described in
 * \todo ref.
 */
struct path_following_force_config final : public rconfig::base_config {
  /**
   * The upper limit for path_following force magnitude.
   */
  double max{0};

  /**
   * The radius around each point along the path to consider as part of the
   * point; i.e., reaching any point in the radius is the same as reaching the
   * exact location of the point.
   */
  double radius{0};
};

NS_END(config, steer2D, cosm);

