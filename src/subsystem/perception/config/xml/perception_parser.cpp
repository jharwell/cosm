/**
 * \file perception_parser.cpp
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
#include "cosm/subsystem/perception/config/xml/perception_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem, perception, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void perception_parser::parse(const ticpp::Element& node) {
  /* Not all robots use a perception subsystem */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ticpp::Element onode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(onode, m_config, los_dim);

  /* grid optional */
  if (nullptr !=
      onode.FirstChild(cds::config::xml::grid2D_parser::kXMLRoot, false)) {
    m_grid.parse(onode);
    m_config->occupancy_grid =
        *m_grid.config_get<cdconfig::xml::grid2D_parser::config_type>();
  }

  /* pheromones optional */
  if (nullptr != onode.FirstChild(pheromone_parser::kXMLRoot, false)) {
    m_pheromone.parse(onode);
    m_config->pheromone =
        *m_pheromone.config_get<pheromone_parser::config_type>();
  }
} /* parse() */

bool perception_parser::validate(void) const {
  if (!is_parsed()) {
    return true;
  }
  return m_grid.validate() && m_pheromone.validate();
} /* validate() */

NS_END(xml, config, perception, subsystem, cosm);
