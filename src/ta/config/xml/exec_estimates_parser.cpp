/**
 * \file exec_estimates_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/config/xml/exec_estimates_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, config, xml);

namespace mxml = rmath::config::xml;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void exec_estimates_parser::parse(const ticpp::Element& node) {
  /* tag is optional in all cases */
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  m_config = std::make_unique<config_type>();
  ticpp::Element enode = node_get(node, kXMLRoot);

  XML_PARSE_ATTR_DFLT(enode, m_config, seed_enabled, false);

  /*
   * This needs to be before the seed enabled check, because even if exec
   * estimate seeding is disabled, we still want to be able to update our
   * estimates.
   */
  m_ema.parse(enode);
  m_config->ema = *m_ema.config_get<mxml::ema_parser::config_type>();

  if (!m_config->seed_enabled) {
    return;
  }
  if (m_task_names.empty()) {
    ER_WARN("No tasks registered for parsing");
  }
  /*
   * For each registered task we want to get exec estimates for, parse the
   * estimate.
   */
  for (auto& s : m_task_names) {
    rmath::rangez tmp{ 0, 0 };
    node_attr_get(enode, s, tmp);
    m_config->ranges.insert({ s, tmp });
  } /* for(&s..) */
} /* parse() */

bool exec_estimates_parser::validate(void) const {
  if (!is_parsed() || !m_config->seed_enabled) {
    return true;
  }
  return m_ema.validate();
} /* validate() */

NS_END(xml, config, ta, cosm);
