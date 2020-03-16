/**
 * \file embodied_block_creator.cpp
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
#include "cosm/pal/embodied_block_creator.hpp"

#include "cosm/repr/ramp_block3D.hpp"
#include "cosm/repr/cube_block3D.hpp"
#include "cosm/pal/argos_sm_adaptor.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
crepr::embodied_block_variant embodied_block_creator::operator()(
    const crepr::cube_block3D* block,
    cpal::argos_sm_adaptor* sm,
    const rmath::radians&) const {
    crepr::embodied_cube_block ret;
    ret.box = new argos::CBoxEntity("cube_block" + rcppsw::to_string(block->id()),
                                    argos::CVector3(block->dloc().x(),
                                                    block->dloc().y(),
                                                    block->dloc().z()),
                                    argos::CQuaternion(),
                                    false,
                                    argos::CVector3(block->dims().x(),
                                                    block->dims().y(),
                                                    block->dims().z()));
    sm->AddEntity(*ret.box);
    return ret;
}

crepr::embodied_block_variant embodied_block_creator::operator()(
    const crepr::ramp_block3D* block,
    cpal::argos_sm_adaptor* sm,
    const rmath::radians& z_rotation) const {
  /*
   * @todo none of the locations/sizes are tuned to back the block look embodied
   * the way it should yet; this is a rough first approximation.
   */
  crepr::embodied_ramp_block ret;
  argos::CQuaternion bottom_orientation;
  bottom_orientation.FromEulerAngles(argos::CRadians(z_rotation.value()),
                                     argos::CRadians::ZERO,
                                     argos::CRadians::ZERO);
  std::string bottom = "ramp_block" + rcppsw::to_string(block->id()) + "_bottom";
  ret.bottom = new argos::CBoxEntity(bottom,
                                     argos::CVector3(block->dloc().x(),
                                                     block->dloc().y(),
                                                     block->dloc().z()),
                                     bottom_orientation,
                                     false,
                                     argos::CVector3(block->dims().x(),
                                                     block->dims().y(),
                                                     0.0001));

  argos::CQuaternion back_orientation;
  back_orientation.FromEulerAngles(argos::CRadians::ZERO,
                                   argos::CRadians::PI_OVER_TWO,
                                   argos::CRadians::ZERO);
  std::string back = "ramp_block" + rcppsw::to_string(block->id()) + "_back";
  ret.bottom = new argos::CBoxEntity(back,
                                     argos::CVector3(block->dloc().x(),
                                                     block->dloc().y(),
                                                     block->dloc().z()),
                                     back_orientation,
                                     false,
                                     argos::CVector3(block->dims().x(),
                                                     block->dims().y(),
                                                     0.0001));
  argos::CQuaternion top_orientation;
  top_orientation.FromEulerAngles(argos::CRadians::ZERO,
                                  argos::CRadians(std::atan2(block->dims().y(),
                                                             block->dims().x())),
                                   argos::CRadians::ZERO);
  std::string top = "ramp_block" + rcppsw::to_string(block->id()) + "_top";
  ret.bottom = new argos::CBoxEntity(top,
                                     argos::CVector3(block->dloc().x(),
                                                     block->dloc().y(),
                                                     block->dloc().z()),
                                     top_orientation,
                                     false,
                                     argos::CVector3(block->dims().x(),
                                                     block->dims().y(),
                                                     0.0001));
  sm->AddEntity(*ret.bottom);
  sm->AddEntity(*ret.back);
  sm->AddEntity(*ret.top);
  return ret;
}

NS_END(pal, cosm);
