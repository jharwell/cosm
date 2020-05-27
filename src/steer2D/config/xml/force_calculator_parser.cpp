/**
 * \file force_calculator_parser.cpp
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
#include "cosm/steer2D/config/xml/force_calculator_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void force_calculator_parser::parse(const ticpp::Element& node) {
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

bool force_calculator_parser::validate(void) const {
  return m_avoidance.validate() && m_arrival.validate() &&
         m_wander.validate() && m_polar.validate() &&
      m_phototaxis.validate() && m_path_following.validate();
} /* validate() */

NS_END(xml, config, steer2D, cosm);
