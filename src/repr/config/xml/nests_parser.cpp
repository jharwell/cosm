/**
 * \file nests_parser.cpp
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
#include "cosm/repr/config/xml/nests_parser.hpp"

#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void nests_parser::parse(const ticpp::Element& node) {
  /*
   * Can be omitted if nests are to be initialized in a way other than from the
   * XML file.
   */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element nnode = node_get(node, kXMLRoot);
  ticpp::Iterator<ticpp::Element> node_it;

  m_config = std::make_unique<config_type>();

  for (node_it = nnode.FirstChildElement(); node_it != node_it.end(); ++node_it) {
    m_nest.parse(*node_it);
    m_config->nests.push_back(*m_nest.config_get<nest_parser::config_type>());
  } /* for(node_it..) */
} /* parse() */

bool nests_parser::validate(void) const {
  if (!is_parsed()) {
    return true;
  }
  for (auto& nest : m_config->nests) {
    ER_CHECK(nest_parser::validate(&nest), "Nest validation failed");
  } /* for(&nest..) */

  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, repr, cosm);
