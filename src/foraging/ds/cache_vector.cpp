/**
 * \file cache_vector.cpp
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
#include "cosm/foraging/ds/cache_vector.hpp"

#include <numeric>

#include "cosm/foraging/repr/arena_cache.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, ds);

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
template <typename TVectorType>
std::string do_to_str(const TVectorType& vec) {
  return std::accumulate(vec.begin(),
                         vec.end(),
                         std::string(),
                         [&](const std::string& a, const auto& c) {
                           return a + "c" + rcppsw::to_string(c->id()) + "@" +
                                  c->dloc().to_str() + ",";
                         });
} /* do_to_str() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string acache_vectoro::to_str(void) const {
  return do_to_str(*this);
} /* to_str() */

std::string acache_vectorno::to_str(void) const {
  return do_to_str(*this);
} /* to_str() */

std::string bcache_vectorno::to_str(void) const {
  return do_to_str(*this);
} /* to_str() */

std::string bcache_vectorro::to_str(void) const {
  return do_to_str(*this);
} /* to_str() */

NS_END(ds, foraging, cosm);
