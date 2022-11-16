/**
 * \file block3D_cluster_vectorro.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/ds/block_cluster_vector.hpp"

#include <numeric>

#include "cosm/foraging/repr/block_cluster.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, ds);

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
template <typename TVector>
std::string do_to_str(const TVector& vec) {
  return std::accumulate(vec.begin(),
                         vec.end(),
                         std::string(),
                         [&](const std::string& a, const auto& c) {
                           return a + "cluster" + rcppsw::to_string(c->id()) +
                                  "@" + c->dcenter2D().to_str() + ",";
                         });
} /* do_to_str() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string block3D_cluster_vectorro::to_str(void) const {
  return do_to_str(decoratee());
} /* to_str() */

std::string block3D_cluster_vectorno::to_str(void) const {
  return do_to_str(decoratee());
} /* to_str() */

NS_END(ds, foraging, cosm);
