/**
 * \file exec_estimates_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <map>
#include <string>

#include "rcppsw/math/config/ema_config.hpp"
#include "rcppsw/math/range.hpp"
#include "rcppsw/config/base_config.hpp"
#include "rcppsw/rcppsw.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct exec_estimates_config
 * \ingroup ta config
 *
 * \brief Parameters for execution time estimates of tasks involved in
 * depth0. Not needed for depth0 controllers, but needed in depth > 1
 * controllers to avoid premature abortion upon start due to estimates of 0.0
 * for generalist task.
 */
struct exec_estimates_config final : public rcppsw::config::base_config {
  /**
   * \brief Should initial estimates of task execution times be used?
   */
  bool seed_enabled{false};
  rmath::config::ema_config ema{};

  std::map<std::string, rmath::rangez> ranges{};
};

NS_END(config, ta, cosm);

