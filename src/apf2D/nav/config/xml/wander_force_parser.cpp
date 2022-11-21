/**
 * \file wander_force_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/apf2D/nav/config/xml/wander_force_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void wander_force_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

    ticpp::Element wnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    XML_PARSE_ATTR(wnode, m_config, interval);
    XML_PARSE_ATTR(wnode, m_config, max);
    XML_PARSE_ATTR(wnode, m_config, circle_distance);
    XML_PARSE_ATTR(wnode, m_config, circle_radius);
    m_bias.parse(wnode);

    m_config->bias_angle = *m_bias.config_get<bias_angle_config>();
  }
} /* parse() */

bool wander_force_parser::validate(void) const {
  if (is_parsed()) {
    ER_CHECK(m_config->circle_distance > 0.0, "Circle distance must be > 0");
    ER_CHECK(m_config->circle_radius > 0.0, "Circle radius must be > 0");
    ER_CHECK(m_config->interval > 0, "Interval must be > 0");
    ER_CHECK(m_config->max >= 0, "Max force must be >= 0");
    ER_CHECK(m_bias.validate(), "Bias angle failed validation");
  }
  return true;

error:
  return false;
} /* validate() */

} /* namespace cosm::apf2D::nav::config::xml */
