/**
 * \file stoch_fov_parser.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: LGPL-2.0-or-later
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/spatial/strategy/flocking/config/xml/stoch_fov_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::spatial::strategy::flocking::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void stoch_fov_parser::parse(const ticpp::Element& node) {
  /* optional */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element fnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(fnode, m_config, strength);
  XML_PARSE_ATTR(fnode, m_config, critical_speed);
  XML_PARSE_ATTR(fnode, m_config, theta_max);
  XML_PARSE_ATTR(fnode, m_config, mean_interaction_dist);


} /* parse() */

} /* namespace cosm::spatial::strategy::flocking::config::xml */
