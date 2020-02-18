/**
 * \file metrics_parser.cpp
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
#include "cosm/metrics/config/xml/metrics_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, metrics, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void metrics_parser::parse(const ticpp::Element& node) {
  /* loop functions metrics not part of controller XML tree  */
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ticpp::Element mnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    XML_PARSE_ATTR(mnode, m_config, output_dir);
    XML_PARSE_ATTR(mnode, m_config, output_interval);

    ticpp::Iterator<ticpp::Attribute> it;
    for (it = mnode.FirstAttribute(); it != it.end(); ++it) {
      if (is_collector_name(*it)) {
        std::string name;
        std::string value;
        it->GetName(&name);
        it->GetValue(&value);
        m_config->enabled.insert({name, value});
      }
    } /* for(i..) */
  }
} /* parse() */

bool metrics_parser::validate(void) const {
  if (is_parsed()) {
    RCSW_CHECK(m_config->output_interval > 0U);
  }
  return true;

error:
  return false;
} /* validate() */

bool metrics_parser::is_collector_name(const ticpp::Attribute& attr) const {
  std::list<std::string> non_names = {"output_dir", "collect_interval"};
  std::string name;
  attr.GetName(&name);
  return non_names.end() == std::find(non_names.begin(), non_names.end(), name);
} /* is_collector_name() */

NS_END(xml, config, metrics, cosm);
