/**
 * \file nest_parser.cpp
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
#include "cosm/repr/config/xml/nest_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void nest_parser::parse(const ticpp::Element& node) {
  /* we do NOT do a node_get() for this parser--rare exception */
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(node, m_config, center);
  XML_PARSE_ATTR(node, m_config, dims);
} /* parse() */

bool nest_parser::validate(void) const {
  return validate(m_config.get());
} /* validate() */

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
bool nest_parser::validate(const nest_config* config) {
  RCSW_CHECK(config->center.is_pd());
  RCSW_CHECK(config->dims.is_pd());
  return true;

error:
  return false;
} /* validate(const nest_config *config)() */

NS_END(xml, config, repr, cosm);
