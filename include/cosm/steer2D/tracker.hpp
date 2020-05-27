/**
 * \file tracker.hpp
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

#ifndef INCLUDE_COSM_STEER2D_TRACKER_HPP_
#define INCLUDE_COSM_STEER2D_TRACKER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <map>
#include <string>
#include <boost/optional.hpp>

#include "cosm/steer2D/ds/path_state.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class tracker
 * \ingroup steer2D
 *
 * \brief Tracker for:
 *
 * - History of applied 2D steering forces
 * - Currently active path(s) the robot is following
 */

class tracker {
 public:
  tracker(void) = default;

  /* Not copy constructable/assignable by default */
  tracker(const tracker&) = delete;
  const tracker& operator=(const tracker&) = delete;

  /**
   * \brief Overwriting add of the the robot's currently active path.
   */
  bool path_add(const ds::path_state& path);

  /**
   * \brief Add the specified force vector to the accumulated vector of forces
   * of the specified category/name.
   */
  bool force_add(const std::string& name, const rmath::vector2d& force);

  /**
   * \brief Return the sum of all the added steering forces of the specified
   * type/name.
   */
  rmath::vector2d force_accum(const std::string& name) const;

  boost::optional<ds::path_state> path(void) const { return m_path; }
  const std::map<std::string, rmath::vector2d>& forces(void) const {
    return m_forces;
  }

  void reset(void) {
    m_path = boost::none;
    m_forces.clear();
  }

 private:
  /* clang-format off */
  boost::optional<ds::path_state>        m_path{};
  std::map<std::string, rmath::vector2d> m_forces{};
  /* clang-format on */
};

NS_END(steer2D, cosm);

#endif /* INCLUDE_COSM_STEER2D_TRACKER_HPP_ */
