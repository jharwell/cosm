/**
 * \file task_alloc_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

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
namespace cosm::ta::config {

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

} /* namespace cosm::ta::config */

