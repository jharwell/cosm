/**
 * \file ucb1_parser.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/config/xml/ucb1_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void ucb1_parser::parse(const ticpp::Element& node) {
  /* executive or policy not used */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  m_config = std::make_unique<config_type>();

  ticpp::Element tnode = node_get(node, kXMLRoot);
  XML_PARSE_ATTR(tnode, m_config, gamma);
} /* parse() */

bool ucb1_parser::validate(void) const {
  return !is_parsed() || m_config->gamma > 0.0;
} /* validate() */

} /* namespace cosm::ta::config::xml */
