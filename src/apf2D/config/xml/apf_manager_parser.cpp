/**
 * \file apf_manager_parser.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/apf2D/config/xml/apf_manager_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void apf_manager_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element knode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_nav.parse(knode);
  m_flocking.parse(knode);

  if (m_nav.is_parsed()) {
    m_config->nav =
        *m_nav.config_get<nav::config::xml::nav_parser::config_type>();
  }
  if (m_flocking.is_parsed()) {
    m_config->flocking =
        *m_flocking.config_get<flocking::config::xml::flocking_parser::config_type>();
  }
} /* parse() */

bool apf_manager_parser::validate(void) const {
  return m_nav.validate() &&
      m_flocking.validate();
} /* validate() */

} /* namespace cosm::apf2D::config::xml */
