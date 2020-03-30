/**
 * \file nest_parser.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "cosm/repr/config/xml/nest_parser.hpp"

#include "rcppsw/utils/line_parser.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr, config, xml);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void nest_parser::parse(const ticpp::Element& node) {
  ticpp::Element nnode = node_get(node, kXMLRoot);

  std::vector<std::string> res;
  rcppsw::utils::line_parser parser(' ');

  res = parser.parse(nnode.GetAttribute("center"));

  m_config = std::make_unique<config_type>();
  m_config->center =
      rmath::vector2d(std::atof(res[0].c_str()), std::atof(res[1].c_str()));
  res = parser.parse(nnode.GetAttribute("size"));
  m_config->dims.x(std::atof(res[0].c_str()));
  m_config->dims.y(std::atof(res[1].c_str()));
} /* parse() */

bool nest_parser::validate(void) const {
  RCSW_CHECK(m_config->center.is_pd());
  RCSW_CHECK(m_config->dims.is_pd());
  return true;

error:
  return false;
} /* validate() */

NS_END(xml, config, repr, cosm);
