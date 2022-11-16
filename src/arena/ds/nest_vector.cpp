/**
 * \file nest_vector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/arena/ds/nest_vector.hpp"

#include <numeric>

#include "cosm/repr/nest.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, ds);

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
template <typename TVector>
std::string do_to_str(const TVector& vec) {
  return std::accumulate(vec.begin(),
                         vec.end(),
                         std::string(),
                         [&](const std::string& a, const auto* n) {
                           return a + n->to_str();
                         });
} /* do_to_str() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string nest_vectorno::to_str(void) const {
  return do_to_str(*this);
} /* to_str() */

std::string nest_vectorro::to_str(void) const {
  return do_to_str(*this);
} /* to_str() */

NS_END(ds, arena, cosm);
