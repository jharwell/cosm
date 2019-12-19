/**
 * \file metrics_parser.cpp
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
#include "cosm/pal/config/xml/metrics_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, pal, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void metrics_parser::parse(const ticpp::Element& node) {
  /* loop functions metrics not part of controller XML tree  */
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ticpp::Element mnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    XML_PARSE_ATTR(mnode, m_config, output_dir);
    XML_PARSE_ATTR(mnode, m_config, output_interval);

    for (auto& m : xml_attr()) {
      if (mnode.HasAttribute(m)) {
        std::string tmp;
        node_attr_get(mnode, m, tmp);
        m_config->enabled[m] = tmp;
      }
    } /* for(&m..) */
  }
} /* parse() */

bool metrics_parser::validate(void) const {
  if (is_parsed()) {
    RCSW_CHECK(m_config->output_interval > 0);
  }
  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, pal, cosm);
