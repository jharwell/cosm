/**
 * \file env_sensor_parser.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/sensors/config/xml/env_sensor_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::hal::sensors::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void env_sensor_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

    ticpp::Element pnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    for (auto& t : m_targets) {
      env_sensor_detection_parser detection(t);
      detection.parse(pnode);
      auto d = detection.config_get<env_sensor_detection_parser::config_type>();
      m_config->detect_map[t] = *d;
    } /* for(&t..) */
  }
} /* parse() */

bool env_sensor_parser::validate(void) const {
  if (!is_parsed()) {
    return true;
  }

  for (auto& d1 : m_config->detect_map) {
    for (auto& d2 : m_config->detect_map) {
      if (d1.first == d2.first) {
        continue;
      }
      ER_CHECK(!d1.second.range.overlaps_with(d2.second.range),
               "Detection range %s->%s overlaps range %s->%s",
               d1.first.c_str(),
               rcppsw::to_string(d1.second.range).c_str(),
               d2.first.c_str(),
               rcppsw::to_string(d2.second.range).c_str());
    } /* for(&d2..) */
  } /* for(&d..) */
  return true;

error:
  return false;
} /* validate() */

} /* namespace cosm::hal::sensors::config::xml */
