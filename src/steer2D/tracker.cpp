/**
 * \file tracker.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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

bool tracker::force_add(const std::string& name,
                        const rutils::color& color,
                        const rmath::vector2d& force) {
  m_forces[name] += map_value_type{ force, color };
  m_forces[name].color = color;
  return true;
} /* force_add() */

void tracker::reset(void) {
  m_path = boost::none;
  for (auto& f : m_forces) {
    f.second = {};
  } /* for(&f..) */
}

NS_END(steer2D, cosm);
