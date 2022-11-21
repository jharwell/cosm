/**
 * \file drop_parser.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/blocks/config/xml/drop_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::spatial::strategy::blocks::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void drop_parser::parse(const ticpp::Element& node) {
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element snode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(snode, m_config, strategy);
  XML_PARSE_ATTR(snode, m_config, duration);
} /* parse() */

} /* namespace cosm::spatial::strategy::blocks::xml, config */
