/**
 * \file task_executive_config.hpp
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

#ifndef INCLUDE_COSM_TA_CONFIG_TASK_EXECUTIVE_CONFIG_HPP_
#define INCLUDE_COSM_TA_CONFIG_TASK_EXECUTIVE_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct task_executive_config
 * \ingroup config ta
 *
 * \brief Configuration for the \ref base_executive and all its derived
 * classes.
 */
struct task_executive_config final : public rcppsw::config::base_config {
  /**
   * \brief Should the executive automatically update execution time estimates
   * for tasks, or will that be handled in the application via callbacks?
   */
  bool update_exec_ests{true};

  /**
   * \brief Should the executive automatically update interface time estimates
   * for tasks, or will that be handled in the application via callbacks?
   */
  bool update_interface_ests{true};
};

NS_END(config, ta, cosm);

#endif /* INCLUDE_COSM_TA_CONFIG_TASK_EXECUTIVE_CONFIG_HPP_ */
