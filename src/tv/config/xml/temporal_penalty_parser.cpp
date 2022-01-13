/**
 * \file temporal_penalty_parser.cpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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
#include "cosm/tv/config/xml/temporal_penalty_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, tv, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void temporal_penalty_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(m_xml_root, false)) {
    ER_DEBUG("Parent node=%s: search for child=%s",
             node.Value().c_str(),
             xml_root().c_str());

    ticpp::Element anode = node_get(node, m_xml_root);
    m_config = std::make_unique<config_type>();

    XML_PARSE_ATTR_DFLT(anode, m_config, unique_finish, true);

    /* parse waveform */
    m_waveform.parse(anode);
    if (m_waveform.is_parsed()) {
      m_config->waveform = *m_waveform.config_get<rct::config::xml::waveform_parser::config_type>();
    }
  }
} /* parse() */

bool temporal_penalty_parser::validate(void) const {
  if (is_parsed()) {
    ER_CHECK(m_waveform.validate(), "Waveform failed validation");
  }
  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, tv, cosm);
