/**
 * \file block_redist_governor_config.hpp
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
#include "cosm/cosm.hpp"

#include "rcppsw/config/base_config.hpp"
#include "rcppsw/types/timestep.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct block_redist_governor_config
 * \ingroup foraging config
 *
 * \brief Configuration for the governor of block redistribution after
 * collection.
 */
struct block_redist_governor_config final : public rconfig::base_config {
  rtypes::timestep timestep{0};
  size_t           block_count{0};
  std::string      disable_trigger{"Null"};
  std::string      recurrence_policy{};
  bool             redistribute{true};
};

NS_END(config, foraging, cosm);
