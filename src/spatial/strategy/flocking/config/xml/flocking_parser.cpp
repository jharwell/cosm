/**
 * \file flocking_parser.cpp
 *
 * \copyright 2022 SIFT LLC, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/flocking/config/xml/flocking_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::spatial::strategy::flocking::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void flocking_parser::parse(const ticpp::Element& node) {
  /* optional */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element fnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  /* mandatory */
  XML_PARSE_ATTR(fnode, m_config, strategy);

  /* optional strategies */
  m_stoch_fov.parse(fnode);

  if (m_stoch_fov.is_parsed()) {
    m_config->stoch_fov =
        *m_stoch_fov.config_get<stoch_fov_parser::config_type>();
  }
} /* parse() */

} /* namespace cosm::spatial::strategy::flocking::config::xml */
