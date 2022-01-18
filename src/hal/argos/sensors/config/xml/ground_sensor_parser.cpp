/**
 * \file ground_sensor_parser.cpp
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
#include "cosm/hal/argos/sensors/config/xml/ground_sensor_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, argos, sensors, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void ground_sensor_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ER_DEBUG("Parent node=%s: child=%s",
             node.Value().c_str(),
             kXMLRoot.c_str());

    ticpp::Element pnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    for (auto& t : m_targets) {
      ground_sensor_detection_parser detection(t);
      detection.parse(pnode);
      auto d =
          detection.config_get<ground_sensor_detection_parser::config_type>();
      m_config->detect_map[t] = *d;
    } /* for(&t..) */
  }
} /* parse() */

bool ground_sensor_parser::validate(void) const {
  for (auto& d1 : m_config->detect_map) {
    for (auto& d2 : m_config->detect_map) {
      if (d1.first == d2.first) {
        continue;
      }
      ER_CHECK(!d1.second.range.overlaps_with(d2.second.range),
               "Detection range %s->%s overlaps range %s->%s",
               d1.first.c_str(),
               rcppsw::to_string(d1.second.range).c_str(),
               d2.first.c_str(),
               rcppsw::to_string(d2.second.range).c_str());
    } /* for(&d2..) */
  } /* for(&d..) */
  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, sensors, argos, hal, cosm);
