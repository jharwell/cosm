/**
 * \file trajectory_parser.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/nav/config/xml/trajectory_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::nav::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void trajectory_parser::parse(const ticpp::Element& node) {
  /* trajectories optional  */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  m_config = std::make_unique<config_type>();
  ticpp::Element tnode = node_get(node, kXMLRoot);

  XML_PARSE_ATTR_DFLT(tnode, m_config, loop, false);
  ticpp::Iterator<ticpp::Element> node_it;
  for (node_it = tnode.FirstChildElement(); node_it != node_it.end(); ++node_it) {
    rmath::vector3d tmp;
    node_attr_get(*node_it, "position", tmp);
    m_config->path.push_back(tmp);
  } /* for(node_it..) */
} /* parse() */

bool trajectory_parser::validate(void) const {
  if (!is_parsed()) {
    return true;
  }
  ER_CHECK(m_config->path.size() > 1, "Path must have at least 2 points");

  return true;

error:
  return false;
} /* validate() */

} /* namespace cosm::nav::config::xml */
