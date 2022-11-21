/**
 * \file visualization_parser.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/argos/vis/config/xml/visualization_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::argos::vis::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void visualization_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

    ticpp::Element vnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    XML_PARSE_ATTR_DFLT(vnode, m_config, robot_id, false);
    XML_PARSE_ATTR_DFLT(vnode, m_config, robot_los, false);
    XML_PARSE_ATTR_DFLT(vnode, m_config, robot_task, false);
    XML_PARSE_ATTR_DFLT(vnode, m_config, robot_apf2D, false);
    XML_PARSE_ATTR_DFLT(vnode, m_config, block_id, false);
  }
} /* parse() */

} /* namespace cosm::argos::vis::config::xml */
