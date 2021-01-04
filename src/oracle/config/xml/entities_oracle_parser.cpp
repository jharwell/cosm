/**
 * \file entities_oracle_parser.cpp
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
#include "cosm/oracle/config/xml/entities_oracle_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, oracle, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void entities_oracle_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ticpp::Element cnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    ticpp::Iterator<ticpp::Attribute> it;
    for (it = cnode.FirstAttribute(); it != it.end(); ++it) {
      std::string name;
      bool value;
      it->GetName(&name);
      it->GetValue(&value);
      m_config->types.insert({ name, value });
    } /* for(it..) */
  }
} /* parse() */

NS_END(xml, config, oracle, cosm);
