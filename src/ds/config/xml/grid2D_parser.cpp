/**
 * \file grid2D_parser.cpp
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
#include "cosm/ds/config/xml/grid2D_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ds, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void grid2D_parser::parse(const ticpp::Element& node) {
  /*
   * May not exist if we are parsing part of an XML tree for perception that
   * does not use grids.
   */
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ticpp::Element gnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    XML_PARSE_ATTR(gnode, m_config, resolution);
    XML_PARSE_ATTR(gnode, m_config, dims);
  }
} /* parse() */

bool grid2D_parser::validate(void) const {
  RCSW_CHECK(m_config->resolution.v() > 0.0);
  RCSW_CHECK(m_config->dims.is_pd());
  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, ds, cosm);
