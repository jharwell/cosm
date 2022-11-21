/**
 * \file aggregate_oracle_parser.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/oracle/config/xml/aggregate_oracle_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::oracle::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void aggregate_oracle_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  /* oracles not used */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ticpp::Element enode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_entities.parse(enode);
  m_tasking.parse(enode);

  if (m_entities.is_parsed()) {
    m_config->entities =
        *m_entities.config_get<entities_oracle_parser::config_type>();
  }
  if (m_tasking.is_parsed()) {
    m_config->tasking =
        *m_tasking.config_get<tasking_oracle_parser::config_type>();
  }
} /* parse() */

bool aggregate_oracle_parser::validate(void) const {
  if (!is_parsed()) {
    return true;
  }
  return m_entities.validate() && m_tasking.validate();
} /* validate() */

} /* namespace cosm::oracle::config::xml */
