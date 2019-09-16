/**
 * @file convergence_config.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_CONVERGENCE_CONFIG_CONVERGENCE_CONFIG_CONFIG_HPP_
#define INCLUDE_COSM_CONVERGENCE_CONFIG_CONVERGENCE_CONFIG_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "cosm/convergence/config/task_dist_entropy_config.hpp"
#include "cosm/convergence/config/positional_entropy_config.hpp"
#include "cosm/convergence/config/interactivity_config.hpp"
#include "cosm/convergence/config/angular_order_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, convergence, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * @struct convergence_config
 * @ingroup cosm convergence config
 */
struct convergence_config final : public rconfig::base_config {
  uint                             n_threads{0};
  double                           epsilon{0};

  struct task_dist_entropy_config  task_dist_entropy{};
  struct positional_entropy_config pos_entropy{};
  struct interactivity_config      interactivity{};
  struct angular_order_config      ang_order{};
};

NS_END(config, convergence, cosm);

#endif /* INCLUDE_COSM_CONVERGENCE_CONFIG_CONVERGENCE_CONFIG_HPP_ */
