/**
 * \file block_carry_visualizer.hpp
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
namespace cosm::repr {
class sim_block3D;
} /* namespace cosm::repr */

NS_START(cosm, argos, vis);

/*******************************************************************************
 * Classes
 ******************************************************************************/
/**
 * \class block_carry_visualizer
 * \ingroup argos vis
 *
 * \brief Renders a block in 3D that the robot is carrying for
 * visualization/debugging purposes.
 */
class block_carry_visualizer : public rer::client<block_carry_visualizer> {
 public:
  block_carry_visualizer(::argos::CQTOpenGLUserFunctions* qt,
                         double block_vis_offset,
                         double text_vis_offset)
      : ER_CLIENT_INIT("cosm.vis.block_carry_visualizer"),
        m_block_vis_offset(block_vis_offset),
        m_text_vis_offset(text_vis_offset),
        m_qt(qt) {}

  block_carry_visualizer(const block_carry_visualizer& op) = delete;
  block_carry_visualizer& operator=(const block_carry_visualizer& op) = delete;

  /**
   * \brief Draw visualizations related to block carries:
   *
   * - The block itself
   * - The block ID
   *
   * \param block The block to draw.
   * \param id_len Length of the robot ID string (to ensure the block ID does
   *               not overlap with it, if it is visualized).
   */
  void draw(const crepr::sim_block3D* block, size_t id_len);

 private:
  /* clang-format off */
  double                                 m_block_vis_offset{0.0};
  double                                 m_text_vis_offset{0.0};
  ::argos::CQTOpenGLUserFunctions* const m_qt{nullptr};
  /* clang-format on */
};

NS_END(vis, argos, cosm);
