/**
 * \file phototaxis_force_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/apf2D/nav/config/xml/phototaxis_force_parser.hpp"

#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::apf2D::nav::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void phototaxis_force_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

    ticpp::Element pnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    XML_PARSE_ATTR(pnode, m_config, max);
  }
} /* parse() */

bool phototaxis_force_parser::validate(void) const {
  if (is_parsed()) {
    ER_CHECK(m_config->max >= 0, "Max force must be >= 0");
  }
  return true;

error:
  return false;
} /* validate() */

} /* namespace cosm::apf2D::nav::config::xml */
