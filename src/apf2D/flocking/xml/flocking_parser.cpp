/**
 * \file flocking_parser.cpp
 *
 * \copyright 2022 SIFT LLC, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/apf2D/flocking/config/xml/flocking_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::flocking::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void flocking_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element knode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_alignment.parse(knode);
  m_constant_speed.parse(knode);

  if (m_alignment.is_parsed()) {
    m_config->alignment =
        *m_alignment.config_get<alignment_force_parser::config_type>();
  }
  if (m_constant_speed.is_parsed()) {
    m_config->constant_speed =
        *m_constant_speed.config_get<constant_speed_force_parser::config_type>();
  }
} /* parse() */

bool flocking_parser::validate(void) const {
  return m_alignment.validate() &&
      m_constant_speed.validate();
} /* validate() */

} /* namespace cosm::apf2D::flocking::config::xml */
