/**
 * \file actuation_subsystem2D_parser.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element anode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_diff_drive.parse(anode);
  m_apf.parse(anode);
  m_config->diff_drive =
      *m_diff_drive
           .config_get<kin2D::config::xml::diff_drive_parser::config_type>();
  m_config->apf_manager = *m_apf.config_get<
      apf2D::config::xml::apf_manager_parser::config_type>();
  ER_DEBUG("Finished");
} /* parse() */

bool actuation_subsystem2D_parser::validate(void) const {
  ER_CHECK(m_diff_drive.validate(), "Diff drive failed validation");
  ER_CHECK(m_apf.validate(), "APF failed validation");

  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, subsystem, cosm);
