/**
 * \file task_alloc_config.hpp
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

#ifndef INCLUDE_COSM_TA_CONFIG_TASK_ALLOC_CONFIG_HPP_
#define INCLUDE_COSM_TA_CONFIG_TASK_ALLOC_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/config/base_config.hpp"
#include "cosm/ta/config/exec_estimates_config.hpp"
#include "cosm/ta/config/src_sigmoid_sel_config.hpp"
#include "cosm/ta/config/stoch_nbhd1_config.hpp"
#include "cosm/ta/config/epsilon_greedy_config.hpp"
#include "cosm/ta/config/ucb1_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct task_alloc_config
 * \ingroup ta config
 *
 * \brief Task allocation configuration container, containing all
 * necessary configuration structs for all possible task allocation policies.
 */
struct task_alloc_config final : public rcppsw::config::base_config {
  /**
   * \brief Policy for specifying how tasks will be allocated in the executive
   * from the data structure containing the tasks to run.
   */
  std::string policy{"random"};

  exec_estimates_config exec_est{};
  src_sigmoid_sel_config abort{};
  stoch_nbhd1_config stoch_nbhd1{};
  epsilon_greedy_config epsilon_greedy{};
  ucb1_config ucb1{};
};

NS_END(config, ta, cosm);

#endif /* INCLUDE_COSM_TA_CONFIG_TASK_ALLOC_CONFIG_HPP_ */
