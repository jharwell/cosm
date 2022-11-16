/**
 * \file visualization_config.hpp
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

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, argos, vis, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct visualization_config
 * \ingroup vis config
 *
 * \brief Configuration for extended ARGoS visualizations.
 */
struct visualization_config : public rconfig::base_config {
  bool robot_id{false};
  bool robot_los{false};
  bool robot_task{false};
  bool robot_apf2D{false};
  bool block_id{false};
};

NS_END(config, vis, argos, cosm);
