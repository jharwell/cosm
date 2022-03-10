/**
 * \file base_controller_repository.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/controller/config/xml/base_controller_repository.hpp"

#include "cosm/subsystem/config/xml/actuation_subsystem2D_parser.hpp"
#include "cosm/hal/subsystem/config/xml/sensing_subsystemQ3D_parser.hpp"
#include "cosm/pal/config/xml/output_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, controller, config, xml);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_controller_repository::base_controller_repository(void) {
  parser_register<csconfig::xml::actuation_subsystem2D_parser,
                  csconfig::actuation_subsystem2D_config>(
                      csconfig::xml::actuation_subsystem2D_parser::kXMLRoot);
  parser_register<chsubsystem::config::xml::sensing_subsystemQ3D_parser,
                  chsubsystem::config::sensing_subsystemQ3D_config>(
                      chsubsystem::config::xml::sensing_subsystemQ3D_parser::kXMLRoot);
  parser_register<cpcxml::output_parser, cpconfig::output_config>(
      cpcxml::output_parser::kXMLRoot);
}

NS_END(xml, config, controller, cosm);
