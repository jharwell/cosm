/**
 * \file task_executive_config.hpp
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

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta::config {

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

} /* namespace cosm::ta::config */

