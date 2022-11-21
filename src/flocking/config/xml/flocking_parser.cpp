/**
 * \file flocking_parser.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/flocking/config/xml/flocking_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::flocking::config::xml {

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
  XML_PARSE_ATTR_DFLT(fnode, m_config, leader_sel_prob, 0.0);

  /* optional strategies */
  m_stoch_fov.parse(fnode);

  /* trajectory optional */
  m_trajectory.parse(fnode);

  if (m_stoch_fov.is_parsed()) {
    m_config->stoch_fov =
        *m_stoch_fov.config_get<stoch_fov_parser::config_type>();
  }
  if (m_trajectory.is_parsed()) {
    m_config->trajectory =
        *m_trajectory.config_get<cnconfig::xml::trajectory_parser::config_type>();
  }
} /* parse() */

} /* namespace cosm::flocking::config::xml */
