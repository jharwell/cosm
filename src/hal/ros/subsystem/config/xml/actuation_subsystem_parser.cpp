/**
 * \file actuation_subsystem_parser.cpp
 *
 * \copyright 2023 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/ros/subsystem/config/xml/actuation_subsystem_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::hal::ros::subsystem::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void actuation_subsystem_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element anode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_apf.parse(anode);
  m_config->apf_manager = *m_apf.config_get<
                          capf2D::config::xml::apf_manager_parser::config_type>();

  m_diff_drive.parse(anode);
  m_config->diff_drive =
      *m_diff_drive
      .config_get<ckin2D::config::xml::diff_drive_parser::config_type>();

  ER_DEBUG("Finished");
} /* parse() */

bool actuation_subsystem_parser::validate(void) const {
  ER_CHECK(m_diff_drive.validate(), "Diff drive failed validation");
  ER_CHECK(m_apf.validate(), "APF failed validation");

  return true;

error:
  return false;
} /* validate() */

} /* namespace cosm::hal::ros::subsystem::config::xml */
