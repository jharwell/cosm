/**
 * \file task_visualizer.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */
#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/er/client.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace argos {
class CQTOpenGLUserFunctions;
}
namespace cosm::ta {
class logical_task;
} /* namespace ta */

namespace cosm::argos::vis {

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class task_visualizer
 * \ingroup argos vis
 *
 * \brief Renders the task that a robot is currently executing in text above the
 * robot for visualization/debugging purposes.
 */
class task_visualizer : public rer::client<task_visualizer> {
 public:
  task_visualizer(::argos::CQTOpenGLUserFunctions* qt, double text_vis_offset)
      : ER_CLIENT_INIT("cosm.vis.task_visualizer"),
        m_text_vis_offset(text_vis_offset),
        m_qt(qt) {}

  task_visualizer(const task_visualizer& op) = delete;
  task_visualizer& operator=(const task_visualizer& op) = delete;

  /**
   * \brief Draw visualizations related to task execution:
   *
   * - The task name
   *
   * \param current_task The current task the robot is executing.
   */
  void draw(const ta::logical_task* current_task);

 private:
  /* clang-format off */
  double                               m_text_vis_offset{0.0};
  ::argos::CQTOpenGLUserFunctions* const m_qt{nullptr};
  /* clang-format on */
};

} /* namespace cosm::argos::vis */

