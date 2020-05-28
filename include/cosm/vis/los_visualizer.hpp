/**
 * \file los_visualizer.hpp
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
#ifndef INCLUDE_COSM_VIS_LOS_VISUALIZER_HPP_
#define INCLUDE_COSM_VIS_LOS_VISUALIZER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/math/vector2.hpp"
#include "rcppsw/math/vector3.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace argos {
class CQTOpenGLUserFunctions;
}

NS_START(cosm, vis);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class los_visualizer
 * \ingroup vis
 *
 * \brief Renders the bounds of a LOS for a given robot for visualization/debugging
 * purposes.
 */
class los_visualizer {
 public:
  explicit los_visualizer(argos::CQTOpenGLUserFunctions* qt) : m_qt(qt) {}

  /* not copy constructible/assignable by default */
  los_visualizer(const los_visualizer&) = delete;
  los_visualizer& operator=(const los_visualizer&) = delete;

  /**
   * \brief Draw the LOS from the robot's current position + vertices.
   */
  void operator()(const std::vector<rmath::vector2d>& points);

  /* clang-format off */
  argos::CQTOpenGLUserFunctions* const m_qt{nullptr};
  /* clang-format on */
};

NS_END(cosm, vis);

#endif /* INCLUDE_COSM_VIS_LOS_VISUALIZER_HPP_ */
