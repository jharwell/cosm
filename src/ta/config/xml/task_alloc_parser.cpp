/**
 * \file task_alloc_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/config/xml/task_alloc_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void task_alloc_parser::parse(const ticpp::Element& node) {
  /* executive not used */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element tnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  /* mandatory */
  XML_PARSE_ATTR(tnode, m_config, policy);

  /* optional, but common to all policies */
  m_estimation.parse(tnode);
  m_abort.parse(node_get(tnode, "task_abort"));

  /* optional policies */
  m_snbhd1.parse(tnode);
  m_epsilon.parse(tnode);
  m_ucb1.parse(tnode);

  m_config->exec_est =
      *m_estimation.config_get<exec_estimates_parser::config_type>();
  m_config->abort = *m_abort.config_get<src_sigmoid_sel_parser::config_type>();

  /* Since these policies are optional, their presence is not guaranteed */
  if (m_snbhd1.is_parsed()) {
    m_config->stoch_nbhd1 =
        *m_snbhd1.config_get<stoch_nbhd1_parser::config_type>();
  }

  if (m_epsilon.is_parsed()) {
    m_config->epsilon_greedy =
        *m_epsilon.config_get<epsilon_greedy_parser::config_type>();
  }

  if (m_ucb1.is_parsed()) {
    m_config->ucb1 = *m_ucb1.config_get<ucb1_parser::config_type>();
  }
} /* parse() */

bool task_alloc_parser::validate(void) const {
  return m_estimation.validate() && m_abort.validate() && m_snbhd1.validate() &&
         m_epsilon.validate() && m_ucb1.validate();
} /* validate() */

} /* namespace cosm::ta::config::xml */
