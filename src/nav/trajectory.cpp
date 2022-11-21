/**
 * \file trajectory.cpp
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <algorithm>

#include "cosm/nav/trajectory.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::nav {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
int trajectory::index_of(const rmath::vector3d& point) const {
  auto it = std::find(m_path.begin(),
                      m_path.end(),
                      point);
  if (it == m_path.end()) {
    return -1;
  }
  return std::distance(m_path.begin(), it);
}

std::string trajectory::to_str(void) const {
  return "path=[" + rcppsw::to_string(m_path) +"]" +
      ",idx=" + rcppsw::to_string(m_index);
} /* to_str() */

} /* namespace cosm::nav */
