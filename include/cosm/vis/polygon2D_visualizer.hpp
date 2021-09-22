/**
 * \file polygon2D_visualizer.hpp
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
#ifndef INCLUDE_COSM_VIS_POLYGON2D_VISUALIZER_HPP_
#define INCLUDE_COSM_VIS_POLYGON2D_VISUALIZER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include <argos3/core/utility/math/quaternion.h>

#include "rcppsw/math/vector2.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/utils/color.hpp"

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
 * \class polygon2D_visualizer
 * \ingroup vis
 *
 * \brief Renders a 2D polygon (including the vertices) in the plane specified
 * by the robot's Z coordinate. Polygons generally are related to a given robot
 * in some way (e.g., a LOS, the nearest known target region, etc.).
 *
 * Rendered polygons can be:
 *
 * - Relative to a robot's current location
 * - Specified in absolute coordinates.
 */
class polygon2D_visualizer {
 public:
  explicit polygon2D_visualizer(argos::CQTOpenGLUserFunctions* qt) : m_qt(qt) {}

  /* not copy constructible/assignable by default */
  polygon2D_visualizer(const polygon2D_visualizer&) = delete;
  polygon2D_visualizer& operator=(const polygon2D_visualizer&) = delete;

  /**
   * \brief Draw the polygon vertices specified in coordinates relative to the
   * robot's current location (i.e., will move as the robot moves).
   *
   * \param pos The robot's current position (needed for Z offset).
   * \param points The points of the polygon, specified in counter clockwise
   *               order.
   * \param color The color of the polygon LINES; vertices are always drawn in
   *               cyan.
   *
   * \p pos and \p orientation are required  to rotate and translate the Qt
   * drawing canvas out of the robot's reference frame.
   */
  void relative_draw(const rmath::vector3d& pos,
                     const std::vector<rmath::vector2d>& points,
                     const rutils::color& color);

  /**
   * \brief Draw the polygon vertices specified in absolute coordinates.
   *
   * \param pos The robot's current position.
   * \param orientation The robot's current orientation.
   * \param points The points of the polygon, specified in counter clockwise
   *               order.
   * \param color The color of the polygon LINES; vertices are always drawn in
   *               cyan.
   *
   * \p pos and \p orientation are required  to rotate and translate the Qt
   * drawing canvas out of the robot's reference frame.
   */
  void abs_draw(const rmath::vector3d& pos,
                const argos::CQuaternion& orientation,
                const std::vector<rmath::vector2d>& points,
                const rutils::color& color);

 private:
  /* draw a little off the ground so it renders better */
  static constexpr const double kDRAW_OFFSET = 0.05;

  /* clang-format off */
  argos::CQTOpenGLUserFunctions* const m_qt{nullptr};
  /* clang-format on */
};

NS_END(cosm, vis);

#endif /* INCLUDE_COSM_VIS_POLYGON2D_VISUALIZER_HPP_ */
