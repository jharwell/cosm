/**
 * \file path_state.hpp
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

#ifndef INCLUDE_COSM_STEER2D_DS_PATH_STATE_HPP_
#define INCLUDE_COSM_STEER2D_DS_PATH_STATE_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <algorithm>

#include "rcppsw/math/vector2.hpp"
#include "rcppsw/patterns/fsm/event.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D, ds);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class path_state
 * \ingroup steer2D ds
 *
 * \brief Holds the overall path for \ref path_following_force, as well as the
 * robot's current progress along it.
 */
class path_state : public rpfsm::event_data {
 public:
  explicit path_state(const std::vector<rmath::vector2d>& points)
      : m_path(points) {}


  const std::vector<rmath::vector2d>& path(void) const { return m_path; }
  rmath::vector2d next_point(void) const { return m_path[m_point_index]; }
  size_t n_points(void) const { return m_path.size(); }
  size_t current_index(void) const { return m_point_index;}
  size_t point_index(const rmath::vector2d& point) const {
    auto it = std::find(m_path.begin(),
                        m_path.end(),
                        point);
    return std::distance(m_path.begin(), it);
  }

  void mark_progress(size_t amount) { m_point_index += amount; }

  bool is_complete(void) const {
    return point_index(next_point()) >= n_points();
  }

  bool operator==(const path_state& other) {
    return m_path == other.m_path;
  }

 private:
  /* clang-format off */
  std::vector<rmath::vector2d> m_path{};

  size_t                       m_point_index{0};
    /* clang-format on */
};

NS_END(ds, steer2D, cosm);

#endif /* INCLUDE_STEER2D_DS_COSM_PATH_STATE_HPP_ */
