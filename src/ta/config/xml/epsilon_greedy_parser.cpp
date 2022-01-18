 /**
 * \file epsilon_greedy_parser.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * This file is part of RCPPSW.
 *
 * RCPPSW is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * RCPPSW is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * RCPPSW.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/config/xml/epsilon_greedy_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void epsilon_greedy_parser::parse(const ticpp::Element& node) {
  /* executive or policy not used */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ER_DEBUG("Parent node=%s: child=%s",
           node.Value().c_str(),
           kXMLRoot.c_str());

  m_config = std::make_unique<config_type>();

  ticpp::Element tnode = node_get(node, kXMLRoot);
  XML_PARSE_ATTR(tnode, m_config, epsilon);
  XML_PARSE_ATTR(tnode, m_config, regret_bound);
} /* parse() */

bool epsilon_greedy_parser::validate(void) const {
  return !is_parsed() || RCPPSW_IS_BETWEEN(m_config->epsilon, 0.0, 1.0);
} /* validate() */

NS_END(xml, config, ta, cosm);
