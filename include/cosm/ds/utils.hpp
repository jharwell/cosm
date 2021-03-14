/**
 * \file utils.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_DS_UTILS_HPP_
#define INCLUDE_COSM_DS_UTILS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <numeric>
#include <vector>
#include <list>
#include <unordered_map>

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::repr {
class base_block3D;
} /* namespace cosm::repr */

NS_START(cosm, ds, detail);

template<typename T, typename U = void>
struct is_mappish_impl : std::false_type { };

template<typename T>
struct is_mappish_impl<T, std::void_t<typename T::key_type,
                                      typename T::mapped_type,
                                      decltype(std::declval<T&>()[std::declval<const typename T::key_type&>()])>>
    : std::true_type { };


template<typename T>
struct is_mappish : detail::is_mappish_impl<T>::type { };

NS_END(detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
template<typename TContainer,
         RCPPSW_SFINAE_FUNC(!detail::is_mappish<TContainer>::value)>
std::string to_string(const TContainer& container, const std::string& prefix) {
  return std::accumulate(container.begin(),
                         container.end(),
                         std::string(),
                         [&](const std::string& a, const auto& b) {
                           return a + prefix + rcppsw::to_string(b->id()) + ",";
                         });
}

template <typename TMap,
          RCPPSW_SFINAE_FUNC(detail::is_mappish<TMap>::value)>
std::string to_string(const TMap& table, const std::string& prefix) {
  return std::accumulate(table.begin(),
                         table.end(),
                         std::string(),
                         [&](const std::string& a, const auto& pair) {
                           return a + prefix + rcppsw::to_string(pair.second->id()) +
                               ",";
                         });
} /* do_to_str() */

NS_END(ds, cosm);

#endif /* INCLUDE_COSM_UTILS_HPP_ */
