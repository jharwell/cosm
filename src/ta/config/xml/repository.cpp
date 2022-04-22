/**
 * \file repository.cpp
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
#include "cosm/ta/config/xml/repository.hpp"

#include "cosm/ta/config/xml/task_alloc_parser.hpp"
#include "cosm/ta/config/xml/task_executive_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ta, config, xml);

/*******************************************************************************
 * Constructors/Destructor
 ******************************************************************************/
repository::repository(void) {
  parser_register<cta::config::xml::task_alloc_parser,
                  cta::config::task_alloc_config>(
      cta::config::xml::task_alloc_parser::kXMLRoot);
  parser_register<cta::config::xml::task_executive_parser,
                  cta::config::task_executive_config>(
      cta::config::xml::task_executive_parser::kXMLRoot);
}

NS_END(xml, config, ta, cosm);
