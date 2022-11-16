/**
 * \file arrival_force_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/apf2D/nav/config/xml/arrival_force_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void arrival_force_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

    ticpp::Element anode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    XML_PARSE_ATTR(anode, m_config, max);
    XML_PARSE_ATTR(anode, m_config, slowing_radius);
    XML_PARSE_ATTR(anode, m_config, slowing_speed_min);
  }
} /* parse() */

bool arrival_force_parser::validate(void) const {
  if (is_parsed()) {
    ER_CHECK(m_config->slowing_radius > 0.0, "Slowing radius must be > 0");
    ER_CHECK(m_config->slowing_speed_min > 0.0, "Slowing min speed must be > 0");
    ER_CHECK(m_config->max > m_config->slowing_speed_min,
             "Max speed must be greater than min slowing speed");
  }
  return true;

error:
  return false;
} /* validate() */

} /* namespace cosm::apf2D::nav::config::xml */
