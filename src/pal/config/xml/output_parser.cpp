/**
 * \file output_parser.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/pal/config/xml/output_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, pal, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void output_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element onode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_metrics_parser.parse(onode);
  if (m_metrics_parser.is_parsed()) {
    m_config->metrics =
        *m_metrics_parser.config_get<rmcxml::metrics_parser::config_type>();
  }

  XML_PARSE_ATTR(onode, m_config, output_parent);
  XML_PARSE_ATTR(onode, m_config, output_leaf);
} /* parse() */

bool output_parser::validate(void) const {
  return m_metrics_parser.validate();
} /* validate() */

NS_END(xml, config, pal, cosm);
