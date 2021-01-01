/**
 * \file steer2D_visualizer.cpp
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
#include "cosm/vis/steer2D_visualizer.hpp"

#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/ray3.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

#include "cosm/steer2D/tracker.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, vis);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void steer2D_visualizer::operator()(const rmath::vector3d& pos,
                                    const argos::CQuaternion& orientation,
                                    const steer2D::tracker* tracker) {
  /* visualize path */
  path_draw(pos, orientation, tracker);

  /* visualize steering forces */
  forces_draw(tracker);
} /* operator()() */

void steer2D_visualizer::path_draw(const rmath::vector3d& pos,
                                   const argos::CQuaternion& orientation,
                                   const steer2D::tracker* tracker) {
  if (auto path = tracker->path()) {
    glPushMatrix();

    /* first, rotate out of the robot's reference frame */
    argos::CRadians x_angle;
    argos::CRadians y_angle;
    argos::CRadians z_angle;

    orientation.ToEulerAngles(z_angle, y_angle, x_angle);
    glRotated(-argos::ToDegrees(z_angle).GetValue(), 0.0, 0.0, 1.0);
    glRotated(-argos::ToDegrees(y_angle).GetValue(), 0.0, 1.0, 0.0);
    glRotated(-argos::ToDegrees(x_angle).GetValue(), 1.0, 0.0, 0.0);

    /* second, translate out of the robot's reference frame */
    glTranslated(-pos.x(), -pos.y(), -pos.z());

    /* Now draw the path segments and end points */
    for (size_t i = 0; i < path->path().size() - 1; ++i) {
      auto start = argos::CVector3(
          path->path()[i].x(), path->path()[i].y(), pos.z() + kDRAW_OFFSET);
      auto end = argos::CVector3(path->path()[i + 1].x(),
                                 path->path()[i + 1].y(),
                                 pos.z() + kDRAW_OFFSET);

      m_qt->DrawRay(argos::CRay3(start, end), argos::CColor::GREEN, 5.0);

      m_qt->DrawPoint(start, argos::CColor::CYAN, 20.0);
      m_qt->DrawPoint(end, argos::CColor::CYAN, 20.0);
    } /* for(i..) */

    /* restore previous OpenGL context */
    glPopMatrix();
  }
} /* path_draw() */

void steer2D_visualizer::forces_draw(const steer2D::tracker* tracker) {
  /* each force gets a ray and a label */
  for (auto& force : tracker->forces()) {
    auto start = argos::CVector3(0.0, 0.0, kDRAW_OFFSET);
    auto end =
        start + argos::CVector3(force.second.x(), force.second.y(), kDRAW_OFFSET);
    m_qt->DrawRay(argos::CRay3(start, end), argos::CColor::MAGENTA, 5.0);
    m_qt->DrawText(argos::CVector3(start.GetX() + end.GetX() / 2.0,
                                   start.GetY() + end.GetY() / 2.0,
                                   start.GetZ() + m_text_vis_offset),
                   force.first,
                   argos::CColor::BLACK);
  } /* for(&force..) */

  /* also draw the accumulated force vector, but in a different color */
  auto accum = std::accumulate(std::begin(tracker->forces()),
                               std::end(tracker->forces()),
                               rmath::vector2d(),
                               [&](const rmath::vector2d& sum, auto& pair) {
                                 return sum + pair.second;
                               });

  auto accum_start = argos::CVector3(0.0, 0.0, kDRAW_OFFSET);
  auto accum_end =
      accum_start + argos::CVector3(accum.x(), accum.y(), kDRAW_OFFSET);
  m_qt->DrawRay(argos::CRay3(accum_start, accum_end), argos::CColor::PURPLE, 5.0);
  m_qt->DrawText(argos::CVector3(accum_start.GetX() + accum_end.GetX() / 2.0,
                                 accum_start.GetY() + accum_end.GetY() / 2.0,
                                 accum_start.GetZ() + m_text_vis_offset),
                 "accum",
                 argos::CColor::BLACK);
} /* forces_draw() */

NS_END(vis, cosm);
