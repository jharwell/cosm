/**
 * \file tracker.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>
#include <map>
#include <string>
#include <vector>

#include "rcppsw/utils/color.hpp"

#include "cosm/nav/trajectory.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class tracker
 * \ingroup apf2D
 *
 * \brief Tracker for:
 *
 * - History of applied 2D APF forces
 * - Currently active path(s) the robot is following
 */

class tracker {
 public:
  struct map_value_type {
    rmath::vector2d force{};
    rutils::color color{};
    map_value_type operator+=(const map_value_type& rhs) {
      force += rhs.force;
      return *this;
    }
  };
  using map_type = std::map<std::string, map_value_type>;

  tracker(void) = default;

  /* Not copy constructable/assignable by default */
  tracker(const tracker&) = delete;
  const tracker& operator=(const tracker&) = delete;

  /**
   * \brief Overwriting add of the the robot's currently active path.
   */
  bool path_add(const cnav::trajectory& path);

  /**
   * \brief Add the specified force vector to the accumulated vector of forces
   * of the specified category/name.
   */
  bool force_add(const std::string& name,
                 const rutils::color& color,
                 const rmath::vector2d& force);

  boost::optional<cnav::trajectory> path(void) const { return m_path; }
  const map_type& forces(void) const { return m_forces; }

  void reset(void);

 private:
  /* clang-format off */
  boost::optional<cnav::trajectory> m_path{};
  map_type                          m_forces{};
  /* clang-format on */
};

} /* namespace cosm::apf2D */
