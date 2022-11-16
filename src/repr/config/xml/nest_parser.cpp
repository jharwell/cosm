/**
 * \file nest_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/repr/config/xml/nest_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void nest_parser::parse(const ticpp::Element& node) {
  ticpp::Element nnode;
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  /* we were called as part of arena configuration */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    nnode = node;
  } else { /* we were called as part of controller configuration */
    nnode = node_get(node, kXMLRoot);
  }
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(nnode, m_config, center);
  XML_PARSE_ATTR(nnode, m_config, dims);
} /* parse() */

bool nest_parser::validate(void) const {
  ER_CHECK(validate(m_config.get()), "Nest failed validation");

  return true;

error:
  return false;
} /* validate() */

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
bool nest_parser::validate(const nest_config* config) {
  RCPPSW_CHECK(config->center.is_pd());
  RCPPSW_CHECK(config->dims.is_pd());
  return true;

error:
  return false;
} /* validate(const nest_config *config)() */

NS_END(xml, config, repr, cosm);
