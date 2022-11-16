/**
 * \file sensing_subsystemQ3D_parser.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/argos/subsystem/config/xml/sensing_subsystemQ3D_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, argos, subsystem, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void sensing_subsystemQ3D_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element snode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_proximity.parse(snode);
  m_config->proximity = *m_proximity.config_get<
      chsensors::config::xml::proximity_sensor_parser::config_type>();
  m_env.parse(snode);
  m_config->env =
      *m_env.config_get<chsensors::config::xml::env_sensor_parser::config_type>();
} /* parse() */

bool sensing_subsystemQ3D_parser::validate(void) const {
  ER_CHECK(m_proximity.validate(), "Proximity sensor failed validation");
  ER_CHECK(m_env.validate(), " Environment sensor failed validation");
  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, subsystem, argos, hal, cosm);
