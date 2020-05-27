/**
 * \file tracker.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
#include "cosm/steer2D/tracker.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
bool tracker::path_add(const ds::path_state& path) {
  m_path = boost::make_optional(path);
  return true;
} /* path_add() */

bool tracker::force_add(const std::string& name, const rmath::vector2d& force) {
  m_forces[name] += force;
  return true;
} /* force_add() */

rmath::vector2d tracker::force_accum(const std::string& name) const {
  auto it = m_forces.find(name);
  if (m_forces.end() == it) {
    return {};
  }
  return it->second;
} /* force_accum() */

NS_END(steer2D, cosm);
