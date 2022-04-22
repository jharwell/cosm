/**
 * \file bias_angle_parser.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
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
