/**
 * \file population_dynamics_parser.cpp
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
#include "cosm/tv/config/xml/population_dynamics_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, tv, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void population_dynamics_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ER_DEBUG("Parent node=%s: search for child=%s",
             node.Value().c_str(),
             kXMLRoot.c_str());

    ticpp::Element anode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    XML_PARSE_ATTR_DFLT(anode, m_config, birth_mu, 0.0);
    XML_PARSE_ATTR_DFLT(anode, m_config, death_lambda, 0.0);
    XML_PARSE_ATTR_DFLT(anode, m_config, malfunction_lambda, 0.0);
    XML_PARSE_ATTR_DFLT(anode, m_config, repair_mu, 0.0);
    XML_PARSE_ATTR_DFLT(anode, m_config, max_size, 0);
  }
} /* parse() */

bool population_dynamics_parser::validate(void) const {
  if (is_parsed()) {
    ER_CHECK(m_config->birth_mu >= 0.0, "Birth mu must be >= 0");
    ER_CHECK(m_config->death_lambda >= 0.0, "Death lambda must be >= 0");
    ER_CHECK(m_config->malfunction_lambda >= 0.0,
             "Malfunction lambda must be >= 0");
    ER_CHECK(m_config->repair_mu >= 0.0, "Repair mu must be >= 0");
  }
  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, tv, cosm);
