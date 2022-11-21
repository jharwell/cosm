/**
 * \file block_manifest_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/config/xml/block_manifest_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::foraging::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_manifest_parser::parse(const ticpp::Element& node) {
  ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

  ticpp::Element bnode = node_get(node, kXMLRoot);
  m_config = std::make_unique<config_type>();

  XML_PARSE_ATTR_DFLT(
      bnode, m_config, n_cube, static_cast<decltype(m_config->n_cube)>(0));
  XML_PARSE_ATTR_DFLT(
      bnode, m_config, n_ramp, static_cast<decltype(m_config->n_ramp)>(0));
  XML_PARSE_ATTR(bnode, m_config, unit_dim);
} /* parse() */

bool block_manifest_parser::validate(void) const {
  ER_CHECK(m_config->unit_dim > 0, "Block dimension must be > 0");
  ER_CHECK(m_config->n_cube > 0 || m_config->n_ramp > 0,
           "Cannot have 0 blocks in manifest");
  return true;

error:
  return false;
} /* validate() */

} /* namespace cosm::foraging::config::xml */
