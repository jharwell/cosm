/**
 * \file aggregate_oracle_parser.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "cosm/oracle/config/xml/aggregate_oracle_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, oracle, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void aggregate_oracle_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  /* oracles not used */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ticpp::Element enode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_entities.parse(enode);
  m_tasking.parse(enode);

  if (m_entities.is_parsed()) {
    m_config->entities =
        *m_entities.config_get<entities_oracle_parser::config_type>();
  }
  if (m_tasking.is_parsed()) {
    m_config->tasking =
        *m_tasking.config_get<tasking_oracle_parser::config_type>();
  }
} /* parse() */

bool aggregate_oracle_parser::validate(void) const {
  if (!is_parsed()) {
    return true;
  }
  return m_entities.validate() && m_tasking.validate();
} /* validate() */

NS_END(xml, config, oracle, cosm);
