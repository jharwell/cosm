/**
 * \file tasking_oracle_config.hpp
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

#ifndef INCLUDE_COSM_ORACLE_CONFIG_TASKING_ORACLE_CONFIG_HPP_
#define INCLUDE_COSM_ORACLE_CONFIG_TASKING_ORACLE_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, oracle, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct tasking_oracle_config
 * \ingroup oracle config
 *
 * \brief Parameters for all-seeing task allocation oracle.
 */
struct tasking_oracle_config final : public rconfig::base_config {
  /**
   * \brief Should the all-knowing oracle be used when updating task execution
   * time estimates? Only applicable to certain controllers.
   */
  bool task_exec_ests{false};

  /**
   * \brief Should the all-knowing oracle be used when updating task interface
   * time estimates? Only applicable to certain controllers.
   */
  bool task_interface_ests{false};
};

NS_END(config, oracle, cosm);

#endif /* INCLUDE_COSM_ORACLE_CONFIG_TASKING_ORACLE_CONFIG_HPP_ */
