/**
 * \file name_spec.cpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/metrics/name_spec.hpp"

#include <regex>

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::metrics::specs {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
name_spec::name_spec(const std::string& xml, const std::string& scoped)
    : m_xml(xml), m_scoped(scoped) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string name_spec::xml(const rtypes::type_uuid& id) const {
  if (rtypes::constants::kNoUUID != id) {
    return std::regex_replace(
        m_xml.c_str(), std::regex("__UUID__"), rcppsw::to_string(id).c_str());
  }
  return m_xml;
} /* xml() */

std::string name_spec::scoped(const rtypes::type_uuid& id) const {
  if (rtypes::constants::kNoUUID != id) {
    return std::regex_replace(
        m_scoped.c_str(), std::regex("__UUID__"), rcppsw::to_string(id).c_str());
  }
  return m_scoped;
} /* scoped() */

} /* namespace cosm::metrics::specs */
