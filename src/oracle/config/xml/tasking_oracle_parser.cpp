/**
 * \file tasking_oracle_parser.cpp
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
#include "cosm/oracle/config/xml/tasking_oracle_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, oracle, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void tasking_oracle_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

    ticpp::Element tonode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    XML_PARSE_ATTR_DFLT(tonode, m_config, task_exec_ests, false);
    XML_PARSE_ATTR_DFLT(tonode, m_config, task_interface_ests, false);
  }
} /* parse() */

NS_END(xml, config, oracle, cosm);
