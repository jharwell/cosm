/**
 * \file diff_drive_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/kin2D/config/xml/diff_drive_parser.hpp"

#include "rcppsw/math/angles.hpp"
#include "rcppsw/math/degrees.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, kin2D, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void diff_drive_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element wnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(wnode, m_config, max_linear_speed);
  XML_PARSE_ATTR(wnode, m_config, max_angular_speed);
  XML_PARSE_ATTR(wnode, m_config, soft_turn_max);
} /* parse() */

bool diff_drive_parser::validate(void) const {
  ER_CHECK(m_config->soft_turn_max.v() > 0.0, "Soft turn max must be > 0");
  ER_CHECK(m_config->max_linear_speed > 0.0, "Max linear speed must be > 0");
  ER_CHECK(m_config->max_angular_speed > 0.0, "Max angular speed must be > 0");
  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, kin2D, cosm);
