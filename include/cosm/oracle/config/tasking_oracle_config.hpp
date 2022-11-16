/**
 * \file tasking_oracle_config.hpp
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
