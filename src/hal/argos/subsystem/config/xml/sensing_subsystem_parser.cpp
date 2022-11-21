/**
 * \file sensing_subsystem_parser.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/argos/subsystem/config/xml/sensing_subsystem_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::hal::argos::subsystem::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void sensing_subsystem_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element snode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_proximity.parse(snode);
  if (m_proximity.is_parsed()) {
    m_config->proximity = *m_proximity.config_get<
      chsensors::config::xml::proximity_sensor_parser::config_type>();
  }

  m_env.parse(snode);
  if (m_env.is_parsed()) {
    m_config->env =
        *m_env.config_get<chsensors::config::xml::env_sensor_parser::config_type>();
  }
} /* parse() */

bool sensing_subsystem_parser::validate(void) const {
  ER_CHECK(m_proximity.validate(), "Proximity sensor failed validation");
  ER_CHECK(m_env.validate(), " Environment sensor failed validation");
  return true;

error:
  return false;
} /* validate() */

} /* namespace cosm::hal::argos::subsystem::config::xml */
