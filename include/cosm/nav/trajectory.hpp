/**
 * \file trajectory.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <string>

#include "rcppsw/math/vector2.hpp"
#include "rcppsw/patterns/fsm/event.hpp"

#include "cosm/cosm.hpp"
#include "cosm/nav/config/trajectory_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::nav {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class trajectory
 * \ingroup nav
 *
 * \brief Trajectory = \ref path3D + tracking where you are along it.
 *
 */
class trajectory : public rpfsm::event_data,
                   public rer::stringizable {
 public:
  explicit trajectory(const config::trajectory_config* config)
      : m_loop(config->loop),
        m_path(config->path) {}

  const path3D& path(void) const { return m_path; }
  size_t size(void) const { return m_path.size(); }

  /**
   * \brief Get the next point along the path.
   */
  rmath::vector3d next_point(void) const { return m_path[m_index]; }

  /**
   * \brief Mark progress along the path by updating the current waypoint.
   */
  void mark_progress(size_t amount) {
    m_index += amount;
    if (m_index >= size() && m_loop) {
      m_index = 0;
    }
  }

  /**
   * \brief Get the index of the specified point along the path.
   *
   * If the point is not on the path, -1 is returned.
   */
  int index_of(const rmath::vector3d& point) const;


  /**
   * \brief Returns \c TRUE iff the path has been completed (no more points) AND
   * we are not looping.
   */

  bool is_complete(void) const {
    return  !m_loop && (-1 == index_of(next_point()));
  }

  std::string to_str(void) const override;

 private:
  /* clang-format off */
  bool   m_loop;
  path3D m_path;

  size_t m_index{0};
    /* clang-format on */
};

} /* namespace cosm::nav */
