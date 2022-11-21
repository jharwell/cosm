/**
 * \file flocking_config.hpp
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
#include <boost/optional.hpp>

#include "rcppsw/config/base_config.hpp"

#include "cosm/cosm.hpp"
#include "cosm/flocking/config/stoch_fov_config.hpp"
#include "cosm/nav/config/trajectory_config.hpp"

/*******************************************************************************
  * Namespaces
******************************************************************************/
namespace cosm::flocking::config {

/*******************************************************************************
  * Structure Definitions
******************************************************************************/
/**
  * \struct flocking_config
  * \ingroup flocking config
  *
  * \brief Configuration for flocking strategies that can be employed by agents.
  */
struct flocking_config final : public rconfig::base_config {
  /**
   * \brief The strategy to employ.
   */
  std::string strategy{rconfig::constants::kNoValue};

  /**
   * \brief The probability that a given agent will be selected as a leader
   * during initialization.
   *
   * This can be used to elect multiple leaders if needed.
   */
  double leader_sel_prob{0};

  /**
   * \brief Configuration for \ref cflocking::stoch_fov.
   */
  stoch_fov_config stoch_fov{};

  /**
   * \brief The trajectory that flocking leaders should follow.
   *
   * Optional, because the particular flocking strategy employed might not need
   * leaders.
   */
  boost::optional<cnconfig::trajectory_config> trajectory{boost::none};
};

} /* namespace cosm::flocking::config */
