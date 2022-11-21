/**
 * \file block_carry_visualizer.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/argos/vis/block_carry_visualizer.hpp"

#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

#include "cosm/repr/sim_block3D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::argos::vis {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_carry_visualizer::draw(const crepr::sim_block3D* const block,
                                  size_t id_len) {
  switch (block->md()->type()) {
    case repr::block_type::ekCUBE:
      m_qt->DrawBox(::argos::CVector3(0.0, 0.0, m_block_vis_offset),
                    ::argos::CQuaternion(),
                    ::argos::CVector3(block->xrsize().v(),
                                      block->xrsize().v(),
                                      block->xrsize().v()),
                    ::argos::CColor(block->md()->color().red(),
                                    block->md()->color().green(),
                                    block->md()->color().blue()));
      break;
    case repr::block_type::ekRAMP:
      /*
       * Ramp blocks are 2X as long in X as in Y. Z height is the same as X (not
       * currently used/handled in the simulation; only for visualization
       * purposes). But, drawing ramp blocks in ARGos is a pain in the ass, so
       * use the spherical cow approach.
       */
      m_qt->DrawBox(::argos::CVector3(0.0, 0.0, m_block_vis_offset),
                    ::argos::CQuaternion(),
                    ::argos::CVector3(block->xrsize().v(),
                                      block->yrsize().v(),
                                      block->yrsize().v()),
                    ::argos::CColor(block->md()->color().red(),
                                    block->md()->color().green(),
                                    block->md()->color().blue()));

      break;
    default:
      /* Unknown block type */
      ER_FATAL_SENTINEL("Cannot visualize unknown block type: Not cube or ramp");
  } /* switch() */

  if (block->vis_id()) {
    m_qt->DrawText(::argos::CVector3(0.0, 0.0, m_text_vis_offset),
                   std::string(id_len + 3, ' ') + "[b" +
                       rcppsw::to_string(block->id()) + "]",
                   ::argos::CColor::GREEN);
  }
} /* draw() */

} /* namespace cosm::argos::vis */
