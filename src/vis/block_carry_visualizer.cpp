/**
 * \file block_carry_visualizer.cpp
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
#include "cosm/vis/block_carry_visualizer.hpp"

#include <argos3/core/utility/datatypes/color.h>
#include <argos3/core/utility/math/quaternion.h>
#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

#include "cosm/repr/base_block2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, vis);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
void block_carry_visualizer::draw(const crepr::base_block2D* const block,
                                  uint id_len) {
  switch (block->md()->type()) {
    case repr::block_type::ekCUBE:
      m_qt->DrawBox(
          argos::CVector3(0.0, 0.0, m_block_vis_offset),
          argos::CQuaternion(),
          argos::CVector3(block->xdimr(), block->xdimr(), block->xdimr()),
          argos::CColor(block->md()->color().red(),
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
      m_qt->DrawBox(
          argos::CVector3(0.0, 0.0, m_block_vis_offset),
          argos::CQuaternion(),
          argos::CVector3(block->xdimr(), block->ydimr(), block->ydimr()),
          argos::CColor(block->md()->color().red(),
                        block->md()->color().green(),
                        block->md()->color().blue()));

      break;
    default:
      /* Unknown block type */
      ER_FATAL_SENTINEL(
          "Cannot visualize unknown block type: Not cube or ramp");
  } /* switch() */

  if (block->vis_id()) {
    m_qt->DrawText(argos::CVector3(0.0, 0.0, m_text_vis_offset),
                   std::string(id_len + 3, ' ') + "[b" +
                       rcppsw::to_string(block->id()) + "]",
                   argos::CColor::GREEN);
  }
} /* draw() */

NS_END(vis, cosm);
