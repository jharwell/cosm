/**
 * \file block_motion_parser.cpp
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
#include "cosm/foraging/config/xml/block_motion_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_motion_parser::parse(const ticpp::Element& node) {
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ER_DEBUG("Parent node=%s: child=%s",
           node.Value().c_str(),
           kXMLRoot.c_str());

  m_config = std::make_unique<config_type>();
  ticpp::Element lnode = node_get(node, kXMLRoot);

  XML_PARSE_ATTR(lnode, m_config, policy);
  XML_PARSE_ATTR_DFLT(lnode, m_config, random_walk_prob, 0.0);
} /* parse() */

bool block_motion_parser::validate(void) const {
  if (is_parsed()) {
    ER_CHECK(RCPPSW_IS_BETWEEN(m_config->random_walk_prob, 0.0, 1.0),
             "Probability must be between 0 and 1");
  }
  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, foraging, cosm);
