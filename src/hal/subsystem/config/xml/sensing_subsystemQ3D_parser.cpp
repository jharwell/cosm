/**
 * \file sensing_subsystemQ3D_parser.cpp
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
#include "cosm/hal/subsystem/config/xml/sensing_subsystemQ3D_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, subsystem, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void sensing_subsystemQ3D_parser::parse(const ticpp::Element& node) {
    ER_DEBUG("Parent node=%s: search for child=%s",
           node.Value().c_str(),
           kXMLRoot.c_str());

    ticpp::Element snode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_proximity.parse(snode);
  m_config->proximity = *m_proximity.config_get<
      chsensors::config::xml::proximity_sensor_parser::config_type>();
  m_env.parse(snode);
  m_config->env = *m_env.config_get<
      chsensors::config::xml::env_sensor_parser::config_type>();
} /* parse() */

bool sensing_subsystemQ3D_parser::validate(void) const {
  ER_CHECK(m_proximity.validate(), "Proximity sensor failed validation");
  ER_CHECK(m_env.validate(), " Environment sensor failed validation");
  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, subsystem, hal, cosm);
