/**
 * \file output_parser.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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
#include "cosm/pal/config/xml/output_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, pal, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void output_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element onode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_metrics_parser.parse(onode);
  if (m_metrics_parser.is_parsed()) {
    m_config->metrics =
        *m_metrics_parser.config_get<rmcxml::metrics_parser::config_type>();
  }

  XML_PARSE_ATTR(onode, m_config, output_parent);
  XML_PARSE_ATTR(onode, m_config, output_leaf);
} /* parse() */

bool output_parser::validate(void) const {
  return m_metrics_parser.validate();
} /* validate() */

NS_END(xml, config, pal, cosm);
