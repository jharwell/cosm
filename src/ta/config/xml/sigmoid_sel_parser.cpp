/**
 * \file sigmoid_sel_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/config/xml/sigmoid_sel_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, config, xml);
namespace mxml = rmath::config::xml;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void sigmoid_sel_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element snode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_sigmoid.parse(snode);
  m_config->sigmoid = *m_sigmoid.config_get<mxml::sigmoid_parser::config_type>();
  XML_PARSE_ATTR(snode, m_config, method);
} /* parse() */

bool sigmoid_sel_parser::validate(void) const {
  ER_CHECK(m_sigmoid.validate(), "Sigmoid failed validation");
  ER_CHECK(!m_config->method.empty(), "Method cannot be empty");
  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, ta, cosm);
