/**
 * \file bias_angle_parser.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/steer2D/config/xml/bias_angle_parser.hpp"

#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void bias_angle_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element wnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(wnode, m_config, src);
  XML_PARSE_ATTR_DFLT(wnode, m_config, max_delta, rmath::radians(-1));

  std::string tmp;
  node_attr_get(wnode, "angles", tmp, std::string());
  m_config->angles =
      rutils::line_parser::as<rmath::radians>(rutils::line_parser(',')(tmp));
} /* parse() */

NS_END(xml, config, steer2D, cosm);
