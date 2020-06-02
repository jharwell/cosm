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

#include "cosm/pal/argos_sm_adaptor.hpp"
#include "cosm/repr/cube_block3D.hpp"
#include "cosm/repr/ramp_block3D.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
crepr::embodied_block_variant embodied_block_creator::operator()(
    const crepr::cube_block3D* block,
    const rmath::radians&,
    const rtypes::type_uuid& parent_id,
    cpal::argos_sm_adaptor* sm) const {
  crepr::embodied_cube_block ret;
  argos::CVector3 loc(block->rpos3D().x(),
                      block->rpos3D().y(),
                      block->rpos3D().z());
  auto name = "parent" + rcppsw::to_string(parent_id) + +"_cube_block" +
              rcppsw::to_string(block->id());
  ret.box = new argos::CBoxEntity(name,
                                  argos::CVector3(block->rpos3D().x(),
                                                  block->rpos3D().y(),
                                                  block->rpos3D().z()),
                                  argos::CQuaternion(),
                                  false,
                                  argos::CVector3(block->dims3D().x(),
                                                  block->dims3D().y(),
                                                  block->dims3D().z()));
  sm->AddEntity(*ret.box);
  return ret;
}

crepr::embodied_block_variant embodied_block_creator::operator()(
    const crepr::ramp_block3D* block,
    const rmath::radians& z_rotation,
    const rtypes::type_uuid& parent_id,
    cpal::argos_sm_adaptor* sm) const {
  crepr::embodied_ramp_block ret;

  /*
   * We approximate a ramp by creating top, bottom, and back, because ARGoS does
   * not (easily) let you create arbitrary polygons, which is necessary for
   * creating the triangular sides. We use very thin boxes for the top, bottom,
   * and back, and this seems to work reasonably well.
   */
  ret.bottom = ramp_bottom(block, z_rotation, parent_id);
  sm->AddEntity(*ret.bottom);

  ret.back = ramp_back(block, z_rotation, parent_id);
  sm->AddEntity(*ret.back);

  ret.top = ramp_top(block, z_rotation, parent_id);
  sm->AddEntity(*ret.top);

  return ret;
}

argos::CBoxEntity* embodied_block_creator::ramp_bottom(
    const crepr::ramp_block3D* block,
    const rmath::radians& z_rotation,
    const rtypes::type_uuid& parent_id) const {
  argos::CQuaternion orientation;
  orientation.FromEulerAngles(argos::CRadians(z_rotation.value()),
                              argos::CRadians::ZERO,
                              argos::CRadians::ZERO);
  std::string name = "parent" + rcppsw::to_string(parent_id) + "_ramp_block" +
                     rcppsw::to_string(block->id()) + "_bottom";
  return new argos::CBoxEntity(name,
                               argos::CVector3(block->rpos3D().x(),
                                               block->rpos3D().y(),
                                               block->rpos3D().z()),
                               orientation,
                               false,
                               argos::CVector3(block->dims3D().x(),
                                               block->dims3D().y(),
                                               kRAMP_BOX_THICKNESS));
} /* ramp_bottom() */

argos::CBoxEntity* embodied_block_creator::ramp_back(
    const crepr::ramp_block3D* block,
    const rmath::radians& z_rotation,
    const rtypes::type_uuid& parent_id) const {
  double x_factor = 0.0;
  double y_factor = 0.0;
  argos::CRadians x_rot = argos::CRadians::ZERO;
  argos::CRadians y_rot = argos::CRadians::ZERO;

  if (rmath::radians::kZERO == z_rotation) {
    x_factor = block->dims3D().x() / 2.0;
    y_rot = argos::CRadians::PI_OVER_TWO;
  } else {
    y_factor = block->dims3D().x() / 2.0;
    x_rot = argos::CRadians::PI_OVER_TWO;
  }
  argos::CVector3 loc(block->rpos3D().x() - x_factor,
                      block->rpos3D().y() - y_factor,
                      block->rpos3D().z());
  argos::CQuaternion orientation;
  orientation.FromEulerAngles(argos::CRadians(z_rotation.value()), y_rot, x_rot);
  std::string name = "parent" + rcppsw::to_string(parent_id) + "_ramp_block" +
                     rcppsw::to_string(block->id()) + "_back";
  return new argos::CBoxEntity(name,
                               loc,
                               orientation,
                               false,
                               argos::CVector3(block->dims3D().x(),
                                               block->dims3D().y(),
                                               kRAMP_BOX_THICKNESS));
} /* ramp_back() */

argos::CBoxEntity* embodied_block_creator::ramp_top(
    const crepr::ramp_block3D* block,
    const rmath::radians& z_rotation,
    const rtypes::type_uuid& parent_id) const {
  double angle = std::atan2(block->dims3D().y(), block->dims3D().x());
  double halfway_height = std::tan(angle) * block->dims3D().x() / 2.0;
  double length = block->dims3D().x() / std::cos(angle);

  argos::CQuaternion orientation;
  argos::CRadians x_rot = argos::CRadians::ZERO;
  argos::CRadians y_rot = argos::CRadians::ZERO;
  if (rmath::radians::kZERO == z_rotation) {
    y_rot = argos::CRadians(angle);
  } else {
    x_rot = -argos::CRadians(angle);
  }
  orientation.FromEulerAngles(argos::CRadians(z_rotation.value()), y_rot, x_rot);
  std::string name = "parent" + rcppsw::to_string(parent_id) + "_ramp_block" +
                     rcppsw::to_string(block->id()) + "_top";

  return new argos::CBoxEntity(
      name,
      argos::CVector3(block->rpos3D().x(),
                      block->rpos3D().y(),
                      block->rpos3D().z() + halfway_height),
      orientation,
      false,
      argos::CVector3(length, block->dims3D().y(), kRAMP_BOX_THICKNESS));
} /* ramp_top() */

NS_END(pal, cosm);
