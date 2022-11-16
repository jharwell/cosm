/**
 * \file stoch_nbhd1_parser.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/config/xml/stoch_nbhd1_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void stoch_nbhd1_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  /* executive or policy not used */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }
  m_config = std::make_unique<config_type>();

  ticpp::Element tnode = node_get(node, kXMLRoot);

  XML_PARSE_ATTR(tnode, m_config, tab_init_policy);

  m_subtask_sel.parse(node_get(tnode, "subtask_sel"));
  m_partitioning.parse(tnode);
  m_tab_sel.parse(node_get(tnode, "tab_sel"));

  m_config->subtask_sel =
      *m_subtask_sel.config_get<src_sigmoid_sel_parser::config_type>();
  m_config->partitioning =
      *m_partitioning.config_get<task_partition_parser::config_type>();
  m_config->tab_sel =
      *m_tab_sel.config_get<src_sigmoid_sel_parser::config_type>();
} /* parse() */

bool stoch_nbhd1_parser::validate(void) const {
  return !is_parsed() || (m_subtask_sel.validate() && m_partitioning.validate() &&
                          m_tab_sel.validate());
} /* validate() */

NS_END(xml, config, ta, cosm);
