/**
 * \file utils.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <numeric>
#include <unordered_map>
#include <vector>

#include "cosm/cosm.hpp"
#include "cosm/ds/detail/mappish.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::repr {
class base_block3D;
} /* namespace cosm::repr */

namespace cosm::ds {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
template <typename TContainer,
          RCPPSW_SFINAE_DECLDEF(!detail::is_mappish<TContainer>::value)>
std::string to_string(const TContainer& container, const std::string& prefix) {
  return std::accumulate(container.begin(),
                         container.end(),
                         std::string(),
                         [&](const std::string& a, const auto& b) {
                           return a + prefix + rcppsw::to_string(b->id()) + ",";
                         });
}

template <typename TMap, RCPPSW_SFINAE_DECLDEF(detail::is_mappish<TMap>::value)>
std::string to_string(const TMap& table, const std::string& prefix) {
  return std::accumulate(table.begin(),
                         table.end(),
                         std::string(),
                         [&](const std::string& a, const auto& pair) {
                           return a + prefix +
                                  rcppsw::to_string(pair.second->id()) + ",";
                         });
} /* do_to_str() */

} /* namespace cosm::ds */
