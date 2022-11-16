/**
 * \file rlos_parser.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/subsystem/perception/config/xml/rlos_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem, perception, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void rlos_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element rnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(rnode, m_config, dim);

  m_grid.parse(rnode);

  if (m_grid.is_parsed()) {
    m_config->grid2D =
        *m_grid.config_get<cdconfig::xml::grid2D_parser::config_type>();
  }
} /* parse() */

NS_END(xml, config, perception, subsystem, cosm);
