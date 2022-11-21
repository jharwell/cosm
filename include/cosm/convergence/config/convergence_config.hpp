/**
 * \file convergence_config.hpp
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
#include "cosm/convergence/config/task_dist_entropy_config.hpp"
#include "cosm/convergence/config/positional_entropy_config.hpp"
#include "cosm/convergence/config/interactivity_config.hpp"
#include "cosm/convergence/config/angular_order_config.hpp"
#include "cosm/convergence/config/velocity_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::convergence::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct convergence_config
 * \ingroup convergence config
 *
 * \brief Container for the configuration of different swarm covergence
 * measures.
 */
struct convergence_config final : public rconfig::base_config {
  uint                             n_threads{0};
  double                           epsilon{0};

  struct task_dist_entropy_config  task_dist_entropy{};
  struct positional_entropy_config pos_entropy{};
  struct interactivity_config      interactivity{};
  struct angular_order_config      ang_order{};
  struct velocity_config           velocity{};
};

} /* namespace cosm::convergence::config */

