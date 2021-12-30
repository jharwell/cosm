/**
 * \file ground_sensor_detection_parser.cpp
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
#include "cosm/hal/argos/sensors/config/xml/ground_sensor_detection_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, argos, sensors, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void ground_sensor_detection_parser::parse(const ticpp::Element& node) {
  ticpp::Element pnode = node_get(node, m_name);
  m_config = std::make_unique<config_type>();
  XML_PARSE_ATTR(pnode, m_config, consensus);
  XML_PARSE_ATTR(pnode, m_config, range);
} /* parse() */

NS_END(xml, config, sensors, argos, hal, cosm);
