/**
 * \file task_dist_entropy_config.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::convergence::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct task_dist_entropy_config
 * \ingroup convergence config
 *
 * \brief Configuration for the task distribution entropy measure, as described
 * in \todo ref here.
 */
struct task_dist_entropy_config final : public rconfig::base_config {
  bool enable{false};
};

} /* namespace cosm::convergence::config */

