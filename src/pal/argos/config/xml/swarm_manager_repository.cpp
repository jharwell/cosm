/**
 * \file swarm_manager_repository.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/pal/argos/config/xml/swarm_manager_repository.hpp"

#include "cosm/arena/config/xml/arena_map_parser.hpp"
#include "cosm/argos/vis/config/xml/visualization_parser.hpp"
#include "cosm/convergence/config/xml/convergence_parser.hpp"
#include "cosm/oracle/config/xml/aggregate_oracle_parser.hpp"
#include "cosm/tv/config/xml/population_dynamics_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, pal, argos, config, xml);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
swarm_manager_repository::swarm_manager_repository(void) noexcept {
  parser_register<caconfig::xml::arena_map_parser, caconfig::arena_map_config>(
      caconfig::xml::arena_map_parser::kXMLRoot);
  parser_register<cavis::config::xml::visualization_parser,
                  cavis::config::visualization_config>(
      cavis::config::xml::visualization_parser::kXMLRoot);

  parser_register<coconfig::xml::aggregate_oracle_parser,
                  coconfig::aggregate_oracle_config>(
      coconfig::xml::aggregate_oracle_parser::kXMLRoot);

  parser_register<cconvergence::config::xml::convergence_parser,
                  cconvergence::config::convergence_config>(
      cconvergence::config::xml::convergence_parser::kXMLRoot);

  parser_register<ctv::config::xml::population_dynamics_parser,
                  ctv::config::population_dynamics_config>(
      ctv::config::xml::population_dynamics_parser::kXMLRoot);
}

NS_END(xml, config, argos, pal, cosm);
