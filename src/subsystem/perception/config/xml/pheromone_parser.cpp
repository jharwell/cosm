/**
 * \file pheromone_parser.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/subsystem/perception/config/xml/pheromone_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem, perception, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void pheromone_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element pnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(pnode, m_config, rho);
  XML_PARSE_ATTR_DFLT(pnode, m_config, repeat_deposit, false);
} /* parse() */

bool pheromone_parser::validate(void) const {
  if (!is_parsed()) {
    return true;
  }
  ER_CHECK(m_config->rho > 0.0, "Decay rate must be > 0");
  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, perception, subsystem, cosm);
