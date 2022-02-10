/**
 * \file stoch_nbhd1_config.hpp
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/config/base_config.hpp"
#include "cosm/ta/config/src_sigmoid_sel_config.hpp"
#include "cosm/ta/config/task_partition_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct stoch_nbhd1_config
 * \ingroup ta config
 *
 * \brief Configuration for the STOCH-NBHD1 task allocation method, as described
 * in \todo: Paper ref.
 */
struct stoch_nbhd1_config final : public rcppsw::config::base_config {
  /**
   * \brief Policy for specifying how the initially active TAB in the executive
   * will be chosen. Valid values are: [root, random, max_depth].
   */
  std::string tab_init_policy{};

  src_sigmoid_sel_config subtask_sel{};
  task_partition_config partitioning{};
  src_sigmoid_sel_config tab_sel{};
};

NS_END(config, ta, cosm);

