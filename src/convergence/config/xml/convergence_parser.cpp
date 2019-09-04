/**
 * @file convergence_parser.cpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/convergence/config/xml/convergence_parser.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, convergence, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void convergence_parser::parse(const ticpp::Element& node) {
  if (nullptr == node.FirstChild(kXMLRoot, false)) {
    return;
  }

  ticpp::Element cnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR(cnode, m_config, n_threads);
  XML_PARSE_ATTR(cnode, m_config, epsilon);

  m_pos_entropy.parse(cnode);
  if (m_pos_entropy.is_parsed()) {
    m_config->pos_entropy =
        *m_pos_entropy.config_get<positional_entropy_parser::config_type>();
  }
  m_task_entropy.parse(cnode);
  if (m_task_entropy.is_parsed()) {
    m_config->task_dist_entropy =
        *m_task_entropy.config_get<task_dist_entropy_parser::config_type>();
  }

  m_interactivity.parse(cnode);
  if (m_interactivity.is_parsed()) {
    m_config->interactivity =
        *m_interactivity.config_get<interactivity_parser::config_type>();
  }

  m_ang_order.parse(cnode);
  if (m_ang_order.is_parsed()) {
    m_config->ang_order =
        *m_ang_order.config_get<angular_order_parser::config_type>();
  }
} /* parse() */

bool convergence_parser::validate(void) const {
  if (!is_parsed()) {
    return true;
  }
  RCSW_CHECK(m_config->n_threads > 0);
  RCSW_CHECK(RCSW_IS_BETWEEN(m_config->epsilon, 0.0, 1.0));
  RCSW_CHECK(m_pos_entropy.validate());
  RCSW_CHECK(m_task_entropy.validate());
  RCSW_CHECK(m_interactivity.validate());
  RCSW_CHECK(m_ang_order.validate());
  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, convergence, cosm);
