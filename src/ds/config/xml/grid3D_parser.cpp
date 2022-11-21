/**
 * \file grid3D_parser.cpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ds/config/xml/grid3D_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::ds::config::xml {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void grid3D_parser::parse(const ticpp::Element& node) {
  /*
   * May not exist if we are parsing part of an XML tree for perception that
   * does not use grids.
   */
  if (nullptr != node.FirstChild(kXMLRoot, false)) {
    ER_DEBUG("Parent node=%s: child=%s", node.Value().c_str(), kXMLRoot.c_str());

    ticpp::Element gnode = node_get(node, kXMLRoot);
    m_config = std::make_unique<config_type>();

    XML_PARSE_ATTR(gnode, m_config, resolution);
    XML_PARSE_ATTR(gnode, m_config, dims);
  }
} /* parse() */

bool grid3D_parser::validate(void) const {
  ER_CHECK(m_config->resolution.v() > 0.0, "Resolution must be > 0");
  ER_CHECK(m_config->dims.is_pd(), "Dimensions must be positive definite");
  return true;

error:
  return false;
} /* validate() */

} /* namespace cosm::ds::config::xml */
