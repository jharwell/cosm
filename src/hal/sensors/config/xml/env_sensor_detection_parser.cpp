/**
 * \file env_sensor_detection_parser.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/sensors/config/xml/env_sensor_detection_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, sensors, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void env_sensor_detection_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), m_name.c_str());

  ticpp::Element pnode = node_get(node, m_name);
  m_config = std::make_unique<config_type>();
  XML_PARSE_ATTR(pnode, m_config, consensus);
  XML_PARSE_ATTR(pnode, m_config, range);
} /* parse() */

NS_END(xml, config, sensors, hal, cosm);
