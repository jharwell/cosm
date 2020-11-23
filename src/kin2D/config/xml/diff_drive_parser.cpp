/**
 * \file diff_drive_parser.cpp
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
#include "cosm/kin2D/config/xml/diff_drive_parser.hpp"

#include "rcppsw/math/angles.hpp"
#include "rcppsw/math/degrees.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, kin2D, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void diff_drive_parser::parse(const ticpp::Element& node) {
  ticpp::Element wnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(wnode, m_config, max_speed);

  rmath::degrees angle;
  node_attr_get(wnode, "soft_turn_max", angle);
  m_config->soft_turn_max = rmath::to_radians(angle);
} /* parse() */

bool diff_drive_parser::validate(void) const {
  RCPPSW_CHECK(m_config->soft_turn_max.v() > 0.0);
  RCPPSW_CHECK(m_config->max_speed > 0.0);
  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, kin2D, cosm);
