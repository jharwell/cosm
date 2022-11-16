/**
 * \file steer2D_visualizer.hpp
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

#include <argos3/core/utility/math/quaternion.h>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/vector3.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace argos {
class CQTOpenGLUserFunctions;
}

namespace cosm::steer2D {
class tracker;
} /* namespace cosm::steer2D */

NS_START(cosm, argos, vis);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class steer2D_visualizer
 * \ingroup argos vis
 *
 * \brief Renders one or more of the following:
 *
 * - The vectors representing individual 2D steering forces active that timestep
 *   for the robot, and their cumulative sum.
 *
 * - The path the robot is currently following, if applicable.
 */
class steer2D_visualizer : public rer::client<steer2D_visualizer> {
 public:
  steer2D_visualizer(::argos::CQTOpenGLUserFunctions* qt, double text_vis_offset)
      : ER_CLIENT_INIT("cosm.vis.steer2D_visualizer"),
        m_text_vis_offset(text_vis_offset),
        m_qt(qt) {}

  steer2D_visualizer(const steer2D_visualizer&) = delete;
  steer2D_visualizer& operator=(const steer2D_visualizer&) = delete;

  void operator()(const rmath::vector3d& pos,
                  const ::argos::CQuaternion& orientation,
                  const steer2D::tracker* tracker,
                  bool with_labels);

 private:
  /* draw a little off the ground so it renders better */
  static constexpr const double kDRAW_OFFSET = 0.05;

  /*
   * Multiply visualized force by a constant so the visualizations can more
   * easily be seen (they are usually pretty small).
   */
  static constexpr const double kVIS_MULTIPLIER = 10.0;

  /**
   * \brief Draw 2D steering force visualizations
   *
   * \param tracker The 2D steering force tracker.
   * \param labels Include textual labels for what the forces are?
   */
  void forces_draw(const steer2D::tracker* tracker,
                   bool labels);

  /**
   * \brief Draw path visualizations
   *
   * \param pos The robot's current position.
   * \param azimuth The robot's current azimuth.
   * \param tracker The 2D steering force tracker.
   */
  void path_draw(const rmath::vector3d& pos,
                 const ::argos::CQuaternion& orientation,
                 const steer2D::tracker* tracker);

  /* clang-format off */
  double                                 m_text_vis_offset{0.0};
  ::argos::CQTOpenGLUserFunctions* const m_qt{nullptr};
  /* clang-format on */
};

NS_END(vis, argos, cosm);

