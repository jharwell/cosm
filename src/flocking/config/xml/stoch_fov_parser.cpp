/**
 * \file stoch_fov_parser.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/flocking/config/xml/stoch_fov_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::flocking::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void stoch_fov_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  /* optional */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ticpp::Element fnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(fnode, m_config, theta_max);
  XML_PARSE_ATTR(fnode, m_config, mean_interaction_dist);
} /* parse() */

} /* namespace cosm::flocking::config::xml */
