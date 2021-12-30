/**
 * \file base_swarm_manager_repository.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include "cosm/pal/config/xml/base_swarm_manager_repository.hpp"

#include "rcppsw/math/config/xml/rng_parser.hpp"

#include "cosm/pal/config/xml/output_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, pal, config, xml);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
base_swarm_manager_repository::base_swarm_manager_repository(void) noexcept {
  parser_register<rmath::config::xml::rng_parser, rmath::config::rng_config>(
      rmath::config::xml::rng_parser::kXMLRoot);

  parser_register<cpconfig::xml::output_parser, cpconfig::output_config>(
      cpconfig::xml::output_parser::kXMLRoot);
}

NS_END(xml, config, pal, cosm);
