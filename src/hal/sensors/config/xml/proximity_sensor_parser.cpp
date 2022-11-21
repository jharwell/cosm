/**
 * \file proximity_sensor_parser.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/sensors/config/xml/proximity_sensor_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::hal::sensors::config::xml {

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
    XML_PARSE_ATTR_DFLT(pnode, m_config, exp_decay, true);
  }
} /* parse() */

bool proximity_sensor_parser::validate(void) const {
  if (!is_parsed()) {
    return true;
  }
  ER_CHECK(m_config->delta > 0.0, "Delta must be > 0");
  ER_CHECK(m_config->fov.lb() < m_config->fov.ub(),
           "Malformed Field Of View range");
  return true;

error:
  return false;
} /* validate() */

} /* namespace cosm::hal::sensors::config::xml */
