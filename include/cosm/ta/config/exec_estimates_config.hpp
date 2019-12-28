/**
 * \file exec_estimates_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

#ifndef INCLUDE_COSM_TA_CONFIG_EXEC_ESTIMATES_CONFIG_HPP_
#define INCLUDE_COSM_TA_CONFIG_EXEC_ESTIMATES_CONFIG_HPP_

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

  std::map<std::string, rmath::range<uint>> ranges{};
};

NS_END(config, ta, cosm);

#endif /* INCLUDE_COSM_TA_CONFIG_EXEC_ESTIMATES_CONFIG_HPP_ */
