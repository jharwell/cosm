/**
 * \file aggregate_oracle_config.hpp
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
#include "cosm/oracle/config/tasking_oracle_config.hpp"
#include "cosm/oracle/config/entities_oracle_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, oracle, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct aggregate_oracle_config
 * \ingroup oracle config
 *
 * \brief Parameters for the various oracles that can be employed during
 * simulation.
 */
struct aggregate_oracle_config final : public rconfig::base_config {
  struct coconfig::tasking_oracle_config tasking{};
  struct coconfig::entities_oracle_config entities{};
};

NS_END(config, oracle, cosm);
