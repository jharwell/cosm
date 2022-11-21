/**
 * \file block_motion_parser.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/config/xml/block_motion_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_motion_parser::parse(const ticpp::Element& node) {
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  m_config = std::make_unique<config_type>();
  ticpp::Element lnode = node_get(node, kXMLRoot);

  XML_PARSE_ATTR(lnode, m_config, policy);
  XML_PARSE_ATTR_DFLT(lnode, m_config, random_walk_prob, 0.0);
} /* parse() */

bool block_motion_parser::validate(void) const {
  if (is_parsed()) {
    ER_CHECK(RCPPSW_IS_BETWEEN(m_config->random_walk_prob, 0.0, 1.0),
             "Probability must be between 0 and 1");
  }
  return true;

error:
  return false;
} /* validate() */

} /* namespace cosm::foraging::config::xml */
