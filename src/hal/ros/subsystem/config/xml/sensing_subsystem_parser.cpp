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
#include "cosm/hal/ros/subsystem/config/xml/sensing_subsystem_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm, hal::ros::subsystem::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void sensing_subsystem_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element snode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_proximity.parse(snode);
  m_config->proximity = *m_proximity.config_get<
      chsensors::config::xml::proximity_sensor_parser::config_type>();

  m_env.parse(snode);
  m_config->env =
      *m_env.config_get<chsensors::config::xml::env_sensor_parser::config_type>();

  m_sonar.parse(snode);
  m_config->sonar = *m_sonar.config_get<
      chrsensors::config::xml::sonar_sensor_parser::config_type>();
} /* parse() */

bool sensing_subsystem_parser::validate(void) const {
  ER_CHECK(m_proximity.validate(), "Proximity sensor failed validation");
  ER_CHECK(m_env.validate(), "Environment sensor failed validation");
  ER_CHECK(m_sonar.validate(), "Sonar sensor failed validation");
  return true;

error:
  return false;
} /* validate() */

} /* namespace cosm::hal::ros::subsystem::xml, config */
