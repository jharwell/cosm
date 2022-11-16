/**
 * \file path_following_force_parser.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/apf2D/nav/config/xml/path_following_force_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void path_following_force_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

    ticpp::Element anode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    XML_PARSE_ATTR(anode, m_config, max);
    XML_PARSE_ATTR(anode, m_config, radius);
  }
} /* parse() */

bool path_following_force_parser::validate(void) const {
  if (is_parsed()) {
    ER_CHECK(m_config->max > 0.0, "Max force must be > 0");
    ER_CHECK(m_config->radius > 0.0, "Radius must be > 0");
  }
  return true;

error:
  return false;
} /* validate() */

} /* namespace cosm::apf2D::nav::config::xml */
