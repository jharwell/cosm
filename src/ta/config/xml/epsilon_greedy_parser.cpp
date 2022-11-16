/**
 * \file epsilon_greedy_parser.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/config/xml/epsilon_greedy_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void epsilon_greedy_parser::parse(const ticpp::Element& node) {
  /* executive or policy not used */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  m_config = std::make_unique<config_type>();

  ticpp::Element tnode = node_get(node, kXMLRoot);
  XML_PARSE_ATTR(tnode, m_config, epsilon);
  XML_PARSE_ATTR(tnode, m_config, regret_bound);
} /* parse() */

bool epsilon_greedy_parser::validate(void) const {
  return !is_parsed() || RCPPSW_IS_BETWEEN(m_config->epsilon, 0.0, 1.0);
} /* validate() */

NS_END(xml, config, ta, cosm);
