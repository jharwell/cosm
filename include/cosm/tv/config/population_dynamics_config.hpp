/**
 * \file population_dynamics_config.hpp
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
#include "rcppsw/control/config/waveform_config.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, tv, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct population_dynamics_config
 * \ingroup tv config
 *
 * \brief Configuration for the swarm population dynamics manager (\ref
 * population_dynamics).
 */
struct population_dynamics_config final : public rconfig::base_config {
  double birth_mu{0.0};
  double death_lambda{0.0};
  double malfunction_lambda{0.0};
  double repair_mu{0.0};
  int max_size{-1}; /* -1 is no limit */
};

NS_END(config, tv, cosm);

