/**
 * \file arena_map_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "cosm/arena/config/xml/arena_map_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void arena_map_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s",
           node.Value().c_str(),
           kXMLRoot.c_str());

  ticpp::Element anode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_grid.parse(anode);
  m_config->grid =
      *m_grid.config_get<cds::config::xml::grid2D_parser::config_type>();

  m_blocks.parse(anode);
  m_config->blocks =
      *m_blocks.config_get<cfconfig::xml::blocks_parser::config_type>();

  m_nests.parse(anode);
  if (m_nests.is_parsed()) {
    m_config->nests =
        *m_nests.config_get<crconfig::xml::nests_parser::config_type>();
  }
} /* parse() */

bool arena_map_parser::validate(void) const {
  return m_grid.validate() && m_blocks.validate() && m_nests.validate();
} /* validate() */

NS_END(xml, config, arena, cosm);
