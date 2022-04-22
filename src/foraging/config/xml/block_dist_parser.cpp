/**
 * \file block_dist_parser.cpp
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
#include "cosm/foraging/config/xml/block_dist_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_dist_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element bnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(bnode, m_config, dist_type);
  XML_PARSE_ATTR_DFLT(bnode, m_config, strict_success, true);

  if ("powerlaw" == m_config->dist_type) {
    m_powerlaw.parse(bnode);
    m_config->powerlaw =
        *m_powerlaw.config_get<powerlaw_dist_parser::config_type>();
  }

  m_manifest.parse(bnode);
  m_config->manifest =
      *m_manifest.config_get<block_manifest_parser::config_type>();

  m_redist_governor.parse(bnode);
  if (m_redist_governor.is_parsed()) {
    m_config->redist_governor =
        *m_redist_governor.config_get<block_redist_governor_parser::config_type>();
  }
} /* parse() */

bool block_dist_parser::validate(void) const {
  return m_powerlaw.validate() && m_manifest.validate() &&
         m_redist_governor.validate();
} /* validate() */

NS_END(xml, config, foraging, cosm);
