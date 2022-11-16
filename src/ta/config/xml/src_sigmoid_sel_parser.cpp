/**
 * \file src_sigmoid_sel_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/config/xml/src_sigmoid_sel_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void src_sigmoid_sel_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element pnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_sigmoid.parse(pnode);
  m_config->sigmoid = *m_sigmoid.config_get<sigmoid_sel_parser::config_type>();

  XML_PARSE_ATTR(pnode, m_config, input_src);
} /* parse() */

bool src_sigmoid_sel_parser::validate(void) const {
  return m_sigmoid.validate();
} /* validate() */

NS_END(xml, config, ta, cosm);
