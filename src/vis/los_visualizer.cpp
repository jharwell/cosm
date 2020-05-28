/**
 * \file los_visualizer.cpp
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
#include "cosm/vis/los_visualizer.hpp"

#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, vis);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void los_visualizer::operator()(const std::vector<rmath::vector2d>& points) {
  std::vector<argos::CVector2> points2 = {
    {points[0].x(), points[0].y()},
    {points[1].x(), points[1].y()},
    {points[2].x(), points[2].y()},
    {points[3].x(), points[3].y()}
  };

  /*
   * Draw LOS slightly above the the specified Z (which is probably a horizontal
   * plane) so that it renders better.
   */
  m_qt->DrawPolygon(argos::CVector3(0.0, 0.0, 0.05),
                    argos::CQuaternion(),
                    points2,
                    argos::CColor::RED,
                    false);
} /* operator()() */

NS_END(vis, cosm);
