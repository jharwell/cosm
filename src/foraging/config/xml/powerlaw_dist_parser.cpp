/**
 * \file powerlaw_dist_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/config/xml/powerlaw_dist_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void powerlaw_dist_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

    ticpp::Element bnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    XML_PARSE_ATTR(bnode, m_config, pwr_min);
    XML_PARSE_ATTR(bnode, m_config, pwr_max);
    XML_PARSE_ATTR(bnode, m_config, n_clusters);
  }
} /* parse() */

bool powerlaw_dist_parser::validate(void) const {
  if (!is_parsed()) {
    return true;
  }
  ER_CHECK(m_config->pwr_min >= 2, "Power must be >= 2");
  ER_CHECK(m_config->pwr_max >= m_config->pwr_min,
           "Max power must be >= min power");
  ER_CHECK(m_config->n_clusters > 0, "# clusters must be > 0");
  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, foraging, cosm);
