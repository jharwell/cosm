/**
 * \file entities_oracle_parser.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/oracle/config/xml/entities_oracle_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, oracle, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void entities_oracle_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

    ticpp::Element cnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    ticpp::Iterator<ticpp::Attribute> it;
    for (it = cnode.FirstAttribute(); it != it.end(); ++it) {
      std::string name;
      bool value;
      it->GetName(&name);
      node_attr_get(cnode, name, value);
      m_config->types.insert({ name, value });
    } /* for(it..) */
  }
} /* parse() */

NS_END(xml, config, oracle, cosm);
