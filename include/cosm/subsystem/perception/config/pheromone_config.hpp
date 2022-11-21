/**
 * \file pheromone_config.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
namespace cosm::subsystem::perception::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct pheromone_config
 * \ingroup subsystem perception config
 *
 * \brief Configuration for robot virtual pheromones.
 */
struct pheromone_config final : public rconfig::base_config {
  double rho{0.0};
  bool repeat_deposit{false};
};

} /* namespace cosm::subsystem::perception::config */

