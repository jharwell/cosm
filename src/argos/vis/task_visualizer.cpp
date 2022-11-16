/**
 * \file task_visualizer.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/argos/vis/task_visualizer.hpp"

#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

#include "cosm/ta/logical_task.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, argos, vis);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void task_visualizer::draw(const ta::logical_task* const current_task) {
  if (nullptr != current_task) {
    m_qt->DrawText(::argos::CVector3(0.0, 0.0, m_text_vis_offset),
                   current_task->name(),
                   ::argos::CColor::BLUE);
  }
}

NS_END(vis, argos, cosm);
