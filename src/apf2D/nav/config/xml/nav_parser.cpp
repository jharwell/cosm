/**
 * \file nav_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/apf2D/nav/config/xml/nav_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void nav_parser::parse(const ticpp::Element& node) {
  /* No return--navigation forces are required. */
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element knode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_avoidance.parse(knode);
  m_arrival.parse(knode);
  m_wander.parse(knode);
  m_polar.parse(knode);
  m_phototaxis.parse(knode);
  m_path_following.parse(knode);

  if (m_avoidance.is_parsed()) {
    m_config->avoidance =
        *m_avoidance.config_get<avoidance_force_parser::config_type>();
  }
  if (m_arrival.is_parsed()) {
    m_config->arrival =
        *m_arrival.config_get<arrival_force_parser::config_type>();
  }
  if (m_wander.is_parsed()) {
    m_config->wander = *m_wander.config_get<wander_force_parser::config_type>();
  }
  if (m_polar.is_parsed()) {
    m_config->polar = *m_polar.config_get<polar_force_parser::config_type>();
  }
  if (m_phototaxis.is_parsed()) {
    m_config->phototaxis =
        *m_phototaxis.config_get<phototaxis_force_parser::config_type>();
  }
  if (m_path_following.is_parsed()) {
    m_config->path_following =
        *m_path_following.config_get<path_following_force_parser::config_type>();
  }
} /* parse() */

bool nav_parser::validate(void) const {
  return m_avoidance.validate() &&
      m_arrival.validate() &&
      m_wander.validate() &&
      m_polar.validate() &&
      m_phototaxis.validate() &&
      m_path_following.validate();
} /* validate() */

} /* namespace cosm::apf2D::nav::config::xml */
