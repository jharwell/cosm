/**
 * \file mappish.hpp
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <unordered_map>

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ds::detail {

/*******************************************************************************
 * Templates
 ******************************************************************************/
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


} /* namespace cosm::ds::detail */
