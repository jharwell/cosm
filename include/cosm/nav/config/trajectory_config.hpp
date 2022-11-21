/**
 * \file trajectory_config.hpp
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
#include "rcppsw/math/vector3.hpp"

#include "cosm/cosm.hpp"
#include "cosm/nav/path.hpp"

/*******************************************************************************
  * Namespaces
******************************************************************************/
namespace cosm::nav::config {

/*******************************************************************************
  * Structure Definitions
******************************************************************************/
/**
  * \struct trajectory_config
  * \ingroup nav config
  *
  * \brief Configuration for a path which an agent/agents should follow.
  */
struct trajectory_config final : public rconfig::base_config {
  /**
   * \brief The path agents will travel along.
   */
  path3D path{};

  /**
   * \brief Should agents should treat the trajectory as infinite?
   *
   * If \c TRUE, always go back to the beginning after getting to the end.
   */
  bool loop{false};
};

} /* namespace cosm::nav::config */
