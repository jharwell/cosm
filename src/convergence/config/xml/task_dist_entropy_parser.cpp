/**
 * \file task_dist_entropy_parser.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/convergence/config/xml/task_dist_entropy_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::convergence::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void task_dist_entropy_parser::parse(const ticpp::Element& node) {
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

    ticpp::Element mnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    XML_PARSE_ATTR(mnode, m_config, enable);
  }
} /* parse() */

} /* namespace cosm::convergence::config::xml */
