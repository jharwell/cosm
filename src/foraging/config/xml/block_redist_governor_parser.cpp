/**
 * \file block_redist_governor_parser.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/config/xml/block_redist_governor_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_redist_governor_parser::parse(const ticpp::Element& node) {
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  m_config = std::make_unique<config_type>();
  ticpp::Element lnode = node_get(node, kXMLRoot);

  XML_PARSE_ATTR(lnode, m_config, redistribute);

  /*
   * All other config ignored, because we currently only have disable trigger
   * configuration with possible re-enabling later, but not the reverse.
   */
  if (!m_config->redistribute) {
    return;
  }
  XML_PARSE_ATTR(lnode, m_config, disable_trigger);
  XML_PARSE_ATTR(lnode, m_config, recurrence_policy);
  XML_PARSE_ATTR_DFLT(lnode, m_config, timestep, rtypes::timestep(0));
  XML_PARSE_ATTR_DFLT(lnode, m_config, block_count, 0UL);
} /* parse() */

NS_END(xml, config, foraging, cosm);
