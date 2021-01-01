/**
 * \file block3D_ht.cpp
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
#include "cosm/ds/block3D_ht.hpp"

#include <numeric>
#include <string>

#include "cosm/repr/base_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ds);

/*******************************************************************************
 * Non-Member Functions
 ******************************************************************************/
template <typename THt>
std::string do_to_str(const THt& table) {
  return std::accumulate(table.begin(),
                         table.end(),
                         std::string(),
                         [&](const std::string& a, const auto& pair) {
                           return a + "b" + rcppsw::to_string(pair.second->id()) +
                                  ",";
                         });
} /* do_to_str() */

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
std::string block3D_hto::to_str(void) const {
  return do_to_str(*this);
} /* to_str() */

std::string block3D_htno::to_str(void) const {
  return do_to_str(*this);
} /* to_str() */

std::string block3D_htro::to_str(void) const {
  return do_to_str(*this);
} /* to_str() */

NS_END(ds, cosm);
