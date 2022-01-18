/**
 * \file wander_force_parser.cpp
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
#include "cosm/steer2D/config/xml/wander_force_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void wander_force_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ER_DEBUG("Parent node=%s: child=%s",
             node.Value().c_str(),
             kXMLRoot.c_str());

    ticpp::Element wnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    XML_PARSE_ATTR(wnode, m_config, interval);
    XML_PARSE_ATTR(wnode, m_config, max);
    XML_PARSE_ATTR(wnode, m_config, circle_distance);
    XML_PARSE_ATTR(wnode, m_config, circle_radius);
    m_bias.parse(wnode);

    m_config->bias_angle = *m_bias.config_get<bias_angle_config>();
  }
} /* parse() */

bool wander_force_parser::validate(void) const {
  if (is_parsed()) {
    ER_CHECK(m_config->circle_distance > 0.0, "Circle distance must be > 0");
    ER_CHECK(m_config->circle_radius > 0.0, "Circle radius must be > 0");
    ER_CHECK(m_config->interval > 0, "Interval must be > 0");
    ER_CHECK(m_config->max > 0, "Max force must be > 0");
    ER_CHECK(m_bias.validate(), "Bias angle failed validation");
  }
  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, steer2D, cosm);
