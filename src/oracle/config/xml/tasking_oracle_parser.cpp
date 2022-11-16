/**
 * \file tasking_oracle_parser.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/oracle/config/xml/tasking_oracle_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, oracle, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void tasking_oracle_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

    ticpp::Element tonode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    XML_PARSE_ATTR_DFLT(tonode, m_config, task_exec_ests, false);
    XML_PARSE_ATTR_DFLT(tonode, m_config, task_interface_ests, false);
  }
} /* parse() */

NS_END(xml, config, oracle, cosm);
