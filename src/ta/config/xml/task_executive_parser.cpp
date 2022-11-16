/**
 * \file task_executive_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/config/xml/task_executive_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void task_executive_parser::parse(const ticpp::Element& node) {
  /* executive not used */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  m_config = std::make_unique<config_type>();

  ticpp::Element pnode = node_get(node, kXMLRoot);
  XML_PARSE_ATTR_DFLT(pnode, m_config, update_exec_ests, false);
  XML_PARSE_ATTR_DFLT(pnode, m_config, update_interface_ests, false);
} /* parse() */

NS_END(xml, config, ta, cosm);
