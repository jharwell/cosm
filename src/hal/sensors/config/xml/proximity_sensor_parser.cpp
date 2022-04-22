/**
 * \file proximity_sensor_parser.cpp
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
#include "cosm/hal/sensors/config/xml/proximity_sensor_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, sensors, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void proximity_sensor_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

    ticpp::Element pnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    XML_PARSE_ATTR(pnode, m_config, delta);
    XML_PARSE_ATTR(pnode, m_config, fov);
  }
} /* parse() */

bool proximity_sensor_parser::validate(void) const {
  ER_CHECK(m_config->delta > 0.0, "Delta must be > 0");
  ER_CHECK(m_config->fov.lb() < m_config->fov.ub(),
           "Malformed Field Of View range");
  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, sensors, hal, cosm);
