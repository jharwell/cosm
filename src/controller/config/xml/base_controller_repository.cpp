/**
 * \file base_controller_repository.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/controller/config/xml/base_controller_repository.hpp"

#include "cosm/hal/subsystem/config/xml/sensing_subsystem_parser.hpp"
#include "cosm/pal/config/xml/output_parser.hpp"
#include "cosm/hal/subsystem/config/xml/actuation_subsystem_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::controller::config::xml {

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_controller_repository::base_controller_repository(void) {
  parser_register<chsubsystem::config::xml::actuation_subsystem_parser,
                  chsubsystem::config::actuation_subsystem_config>(
                      chsubsystem::config::xml::actuation_subsystem_parser::kXMLRoot);
  parser_register<chsubsystem::config::xml::sensing_subsystem_parser,
                  chsubsystem::config::sensing_subsystem_config>(
      chsubsystem::config::xml::sensing_subsystem_parser::kXMLRoot);
  parser_register<cpcxml::output_parser, cpconfig::output_config>(
      cpcxml::output_parser::kXMLRoot);
}

} /* namespace cosm::controller::config::xml */
