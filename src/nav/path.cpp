/**
 * \file path.cpp
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/nav/path.hpp"

#include <numeric>

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::nav {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string path3D::to_str(void) const {
  return std::accumulate(decoratee().begin(),
                         decoratee().end(),
                         std::string(),
                         [&](const std::string& a, const auto& b) {
                           return a + "pt=" + rcppsw::to_string(b) + ",";
                         });
} /* to_str() */

} /* namespace cosm::nav */
