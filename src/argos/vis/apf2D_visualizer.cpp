/**
 * \file apf2D_visualizer.cpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/argos/vis/apf2D_visualizer.hpp"

#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/ray3.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

#include "cosm/apf2D/tracker.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::argos::vis {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void apf2D_visualizer::operator()(const rmath::vector3d& pos,
                                    const ::argos::CQuaternion& orientation,
                                    const apf2D::tracker* tracker,
                                    bool with_labels) {
  /* visualize path */
  path_draw(pos, orientation, tracker);

  /* visualize steering forces */
  forces_draw(tracker, with_labels);
} /* operator()() */

void apf2D_visualizer::path_draw(const rmath::vector3d& pos,
                                   const ::argos::CQuaternion& orientation,
                                   const apf2D::tracker* tracker) {
  if (auto path = tracker->path()) {
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

    /* Now draw the path segments and end points */
    for (size_t i = 0; i < path->path().size() - 1; ++i) {
      auto start = ::argos::CVector3(
          path->path()[i].x(), path->path()[i].y(), pos.z() + kDRAW_OFFSET);
      auto end = ::argos::CVector3(path->path()[i + 1].x(),
                                   path->path()[i + 1].y(),
                                   pos.z() + kDRAW_OFFSET);

      m_qt->DrawRay(::argos::CRay3(start, end), ::argos::CColor::GREEN, 5.0);

      m_qt->DrawPoint(start, ::argos::CColor::CYAN, 20.0);
      m_qt->DrawPoint(end, ::argos::CColor::CYAN, 20.0);
    } /* for(i..) */

    /* restore previous OpenGL context */
    glPopMatrix();
  }
} /* path_draw() */

void apf2D_visualizer::forces_draw(const apf2D::tracker* tracker,
                                     bool labels) {
  /* each force gets a ray and a label */
  rmath::vector2d accum;
  for (auto& force : tracker->forces()) {
    if (force.second.force.length() <= 0.0) {
      continue;
    }
    auto start = ::argos::CVector3(0.0, 0.0, kDRAW_OFFSET);

    accum += force.second.force * kVIS_MULTIPLIER;
    auto end = start + ::argos::CVector3(force.second.force.x() * kVIS_MULTIPLIER,
                                         force.second.force.y() * kVIS_MULTIPLIER,
                                         kDRAW_OFFSET);
    m_qt->DrawRay(::argos::CRay3(start, end),
                  ::argos::CColor(force.second.color.red(),
                                  force.second.color.green(),
                                  force.second.color.blue()),
                  5.0);
    if (labels) {
      m_qt->DrawText(::argos::CVector3(start.GetX() + end.GetX() / 2.0,
                                       start.GetY() + end.GetY() / 2.0,
                                       start.GetZ() + m_text_vis_offset),
                     force.first,
                     ::argos::CColor::BLACK);
    }

  } /* for(&force..) */

  /* also draw the accumulated force vector, but in a different color */
  /* auto accum_start = ::argos::CVector3(0.0, 0.0, kDRAW_OFFSET); */
  /* auto accum_end = accum_start + ::argos::CVector3(accum.x(), */
  /*                                                accum.y(), */
  /*                                                kDRAW_OFFSET); */
  /* m_qt->DrawRay(::argos::CRay3(accum_start, accum_end), */
  /*               ::argos::CColor::PURPLE, */
  /*               5.0); */
  /* m_qt->DrawText(::argos::CVector3(accum_start.GetX() + accum_end.GetX() / 2.0, */
  /*                                accum_start.GetY() + accum_end.GetY() / 2.0, */
  /*                                accum_start.GetZ() + m_text_vis_offset), */
  /*                "accum", */
  /*                ::argos::CColor::BLACK); */
} /* forces_draw() */

} /* namespace cosm::argos::vis */
