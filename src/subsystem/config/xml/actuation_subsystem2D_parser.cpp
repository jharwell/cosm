/**
 * \file actuation_subsystem2D_parser.cpp
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
#include "cosm/subsystem/config/xml/actuation_subsystem2D_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void actuation_subsystem2D_parser::parse(const ticpp::Element& node) {
  ticpp::Element anode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_diff_drive.parse(anode);
  m_steering.parse(anode);
  m_config->diff_drive =
      *m_diff_drive
           .config_get<kin2D::config::xml::diff_drive_parser::config_type>();
  m_config->steering = *m_steering.config_get<
      steer2D::config::xml::force_calculator_parser::config_type>();
} /* parse() */

bool actuation_subsystem2D_parser::validate(void) const {
  return m_diff_drive.validate() && m_steering.validate();
} /* validate() */

NS_END(xml, config, subsystem, cosm);
