/**
 * \file block_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/config/xml/blocks_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void blocks_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element bnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_dist.parse(bnode);
  m_motion.parse(bnode);
  m_config->dist = *m_dist.config_get<block_dist_parser::config_type>();

  if (m_motion.is_parsed()) {
    m_config->motion = *m_motion.config_get<block_motion_parser::config_type>();
  }
} /* parse() */

bool blocks_parser::validate(void) const {
  return m_dist.validate();
} /* validate() */

} /* namespace cosm::foraging::config::xml */
