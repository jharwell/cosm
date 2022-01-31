/**
 * \file polygon2D_visualizer.cpp
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
#include "cosm/argos/vis/polygon2D_visualizer.hpp"

#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, argos, vis);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void polygon2D_visualizer::relative_draw(
    const rmath::vector3d& pos,
    const std::vector<rmath::vector2d>& points,
    const rutils::color& color) {
  for (size_t i = 0; i < points.size(); ++i) {
    auto start =
        ::argos::CVector3(points[i].x(), points[i].y(), pos.z() + kDRAW_OFFSET);
    /* modulo so that the last vertex can be connected to the first one */
    auto end = ::argos::CVector3(points[(i + 1) % points.size()].x(),
                               points[(i + 1) % points.size()].y(),
                               pos.z() + kDRAW_OFFSET);

    /* draw segment line */
    m_qt->DrawRay(::argos::CRay3(start, end),
                  ::argos::CColor(color.red(), color.green(), color.blue()),
                  5.0);

    /* draw segment endpoints */
    m_qt->DrawPoint(start, ::argos::CColor::CYAN, 20.0);
    m_qt->DrawPoint(end, ::argos::CColor::CYAN, 20.0);
  } /* for(i..) */
} /* relative_draw() */

void polygon2D_visualizer::abs_draw(const rmath::vector3d& pos,
                                    const ::argos::CQuaternion& orientation,
                                    const std::vector<rmath::vector2d>& points,
                                    const rutils::color& color) {
  glPushMatrix();

  /* first, rotate out of the robot's reference frame */
  ::argos::CRadians x_angle;
  ::argos::CRadians y_angle;
  ::argos::CRadians z_angle;

  orientation.ToEulerAngles(z_angle, y_angle, x_angle);
  glRotated(-::argos::ToDegrees(z_angle).GetValue(), 0.0, 0.0, 1.0);
  glRotated(-::argos::ToDegrees(y_angle).GetValue(), 0.0, 1.0, 0.0);
  glRotated(-::argos::ToDegrees(x_angle).GetValue(), 1.0, 0.0, 0.0);

  /* second, translate out of the robot's reference frame */
  glTranslated(-pos.x(), -pos.y(), -pos.z());

  /* Now draw the polygon in 2D */
  for (size_t i = 0; i < points.size(); ++i) {
    auto start =
        ::argos::CVector3(points[i].x(), points[i].y(), pos.z() + kDRAW_OFFSET);
    /* modulo so that the last vertex can be connected to the first one */
    auto end = ::argos::CVector3(points[(i + 1) % points.size()].x(),
                               points[(i + 1) % points.size()].y(),
                               pos.z() + kDRAW_OFFSET);

    /* draw segment line */
    m_qt->DrawRay(::argos::CRay3(start, end),
                  ::argos::CColor(color.red(), color.green(), color.blue()),
                  5.0);

    /* draw segment endpoints */
    m_qt->DrawPoint(start, ::argos::CColor::CYAN, 20.0);
    m_qt->DrawPoint(end, ::argos::CColor::CYAN, 20.0);
  } /* for(i..) */

  /* restore previous OpenGL context */
  glPopMatrix();
} /* abs_draw() */

NS_END(vis, argos, cosm);
