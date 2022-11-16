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

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::repr {
class base_block3D;
} /* namespace cosm::repr */

NS_START(cosm, ds, detail);

template <typename T, typename U = void>
struct is_mappish_impl : std::false_type {};

template <typename T>
struct is_mappish_impl<T,
                       std::void_t<typename T::key_type,
                                   typename T::mapped_type,
                                   decltype(std::declval<T&>()[std::declval<
                                       const typename T::key_type&>()])>>
    : std::true_type {};

template <typename T>
struct is_mappish : detail::is_mappish_impl<T>::type {};

NS_END(detail);

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

NS_END(ds, cosm);
