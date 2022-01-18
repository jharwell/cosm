/**
 * \file block_parser.cpp
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
#include "cosm/foraging/config/xml/blocks_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void blocks_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s",
           node.Value().c_str(),
           kXMLRoot.c_str());

  ticpp::Element bnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_dist.parse(bnode);
  m_motion.parse(bnode);
  m_config->dist = *m_dist.config_get<block_dist_parser::config_type>();

  if (m_motion.is_parsed()) {
    m_config->motion = *m_motion.config_get<block_motion_parser::config_type>();
  }
} /* parse() */

bool blocks_parser::validate(void) const {
  return m_dist.validate();
} /* validate() */

NS_END(xml, config, foraging, cosm);
