/**
 * \file explore_parser.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/explore/config/xml/explore_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, strategy, explore, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void explore_parser::parse(const ticpp::Element& node) {
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element snode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(snode, m_config, strategy);
  XML_PARSE_ATTR_DFLT(snode, m_config, min_duration, rtypes::timestep(1));
} /* parse() */

NS_END(xml, config, explore, strategy, spatial, cosm);
