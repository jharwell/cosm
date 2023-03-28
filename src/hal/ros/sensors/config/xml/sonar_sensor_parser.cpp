/**
 * \file sonar_sensor_parser.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/ros/sensors/config/xml/sonar_sensor_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::hal::ros::sensors::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void sonar_sensor_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

    ticpp::Element pnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    XML_PARSE_ATTR_DFLT(pnode, m_config, trigger_pin, -1);
    XML_PARSE_ATTR_DFLT(pnode, m_config, echo_pin, -1);
  }
} /* parse() */

bool sonar_sensor_parser::validate(void) const {
  ER_CHECK(m_config->trigger_pin > 0, "Trigger pin must be defined");
  ER_CHECK(m_config->echo_pin > 0, "Echo pin must be defined");
  return true;

error:
  return false;
} /* validate() */

} /* namespace cosm::hal::ros::sensors::xml, config */
