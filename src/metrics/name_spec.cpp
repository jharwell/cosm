/**
 * \file name_spec.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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
#include "cosm/metrics/name_spec.hpp"

#include <regex>

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, metrics, specs);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
name_spec::name_spec(const std::string& xml,
                     const std::string& scoped)
    : m_xml(xml),
      m_scoped(scoped) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string name_spec::xml(const rtypes::type_uuid& id) const {
  if (rtypes::constants::kNoUUID != id) {
    return std::regex_replace(m_xml.c_str(),
                               std::regex("__UUID__"),
                               rcppsw::to_string(id).c_str());
  }
  return m_xml;
} /* xml() */

std::string name_spec::scoped(const rtypes::type_uuid& id) const {
  if (rtypes::constants::kNoUUID != id) {
    return std::regex_replace(m_scoped.c_str(),
                               std::regex("__UUID__"),
                               rcppsw::to_string(id).c_str());
  }
  return m_scoped;
} /* scoped() */

NS_END(specs, metrics, cosm);
