/**
 * \file base_swarm_manager_repository.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/pal/config/xml/base_swarm_manager_repository.hpp"

#include "rcppsw/math/config/xml/rng_parser.hpp"

#include "cosm/pal/config/xml/output_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::pal::config::xml {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_swarm_manager_repository::base_swarm_manager_repository(void) noexcept {
  parser_register<rmath::config::xml::rng_parser, rmath::config::rng_config>(
      rmath::config::xml::rng_parser::kXMLRoot);

  parser_register<cpconfig::xml::output_parser, cpconfig::output_config>(
      cpconfig::xml::output_parser::kXMLRoot);
}

} /* namespace cosm::pal::config::xml */
