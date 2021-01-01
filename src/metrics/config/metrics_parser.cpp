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
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ticpp::Element mnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(mnode, m_config, output_dir);

  if (nullptr != mnode.FirstChild("create", false)) {
    output_mode_parse(node_get(mnode, "create"), &m_config->create);
  }
  if (nullptr != mnode.FirstChild("append", false)) {
    output_mode_parse(node_get(mnode, "append"), &m_config->append);
  }
  if (nullptr != mnode.FirstChild("truncate", false)) {
    output_mode_parse(node_get(mnode, "truncate"), &m_config->truncate);
  }
} /* parse() */

void metrics_parser::output_mode_parse(const ticpp::Element& element,
                                       metrics_output_mode_config* config) {
  XML_PARSE_ATTR(element, config, output_interval);
  ticpp::Iterator<ticpp::Attribute> it;
  for (it = element.FirstAttribute(); it != it.end(); ++it) {
    if (is_collector_name(*it)) {
      std::string name;
      std::string value;
      it->GetName(&name);
      it->GetValue(&value);
      config->enabled.insert({ name, value });
    }
  } /* for(it..) */
} /* output_mode_parse() */

bool metrics_parser::is_collector_name(const ticpp::Attribute& attr) const {
  std::list<std::string> non_names = { "collect_interval" };
  std::string name;
  attr.GetName(&name);
  return non_names.end() == std::find(non_names.begin(), non_names.end(), name);
} /* is_collector_name() */

NS_END(xml, config, metrics, cosm);
