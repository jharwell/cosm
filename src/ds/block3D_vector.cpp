/**
 * \file block3D_vector.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include "cosm/ds/block3D_vector.hpp"

#include <numeric>

#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ds);

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
template <typename TVector>
std::string do_to_str(const TVector& vec) {
  return std::accumulate(vec.begin(),
                         vec.end(),
                         std::string(),
                         [&](const std::string& a, const auto& b) {
                           return a + "b" + rcppsw::to_string(b->id()) + ",";
                         });
} /* do_to_str() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string block3D_vectoro::to_str(void) const {
  return do_to_str(*this);
} /* to_str() */

std::string block3D_vectorno::to_str(void) const {
  return do_to_str(*this);
} /* to_str() */

std::string block3D_vectorro::to_str(void) const {
  return do_to_str(*this);
} /* to_str() */

NS_END(ds, cosm);
