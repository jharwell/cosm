/**
 * \file grid_parser.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/subsystem/perception/rlos/config/xml/grid_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem::perception::rlos::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void grid_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  /* optional */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ticpp::Element rnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(rnode, m_config, dim);

  m_grid.parse(rnode);

  if (m_grid.is_parsed()) {
    m_config->grid2D =
        *m_grid.config_get<cdconfig::xml::grid2D_parser::config_type>();
  }
} /* parse() */

} /* namespace cosm::subsystem::perception::rlos::config::xml */
