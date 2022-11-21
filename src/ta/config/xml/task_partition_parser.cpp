/**
 * \file task_partition_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/config/xml/task_partition_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void task_partition_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element pnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  m_sigmoid.parse(pnode);
  m_config->src_sigmoid =
      *m_sigmoid.config_get<src_sigmoid_sel_parser::config_type>();

  XML_PARSE_ATTR_DFLT(pnode, m_config, always_partition, false);
  XML_PARSE_ATTR_DFLT(pnode, m_config, never_partition, false);
} /* parse() */

bool task_partition_parser::validate(void) const {
  if (is_parsed()) {
    ER_CHECK(m_sigmoid.validate(), "Sigmoid failed validation");
  }
  return true;

error:
  return false;
} /* validate() */

} /* namespace cosm::ta::config::xml */
