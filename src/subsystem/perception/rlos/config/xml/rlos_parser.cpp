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
#include "cosm/subsystem/perception/rlos/config/xml/rlos_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::subsystem::perception::rlos::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void rlos_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  /* optional */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ticpp::Element rnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_grid.parse(rnode);
  if (m_grid.is_parsed()) {
    m_config->grid = *m_grid.config_get<grid_parser::config_type>();
  }

  m_fov.parse(rnode);
  if (m_fov.is_parsed()) {
    m_config->fov = *m_fov.config_get<fov_parser::config_type>();
  }
} /* parse() */

} /* namespace cosm::subsystem::perception::rlos::config::xml */
