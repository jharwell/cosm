/**
 * \file dpo_parser.cpp
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
#include "cosm/subsystem/perception/config/xml/dpo_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem, perception, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void dpo_parser::parse(const ticpp::Element& node) {
  /* DPO perception not used */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }
  ticpp::Element dnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_pheromone.parse(dnode);
  m_config->pheromone = *m_pheromone.config_get<pheromone_parser::config_type>();
} /* parse() */

bool dpo_parser::validate(void) const {
  if (!is_parsed()) {
    return true;
  }
  RCPPSW_CHECK(m_pheromone.validate());

  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, perception, subsystem, cosm);
