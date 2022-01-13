/**
 * \file block_embodiment_creator.hpp
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

#ifndef INCLUDE_COSM_PAL_ARGOS_BLOCK_EMBODIMENT_CREATOR_HPP_
#define INCLUDE_COSM_PAL_ARGOS_BLOCK_EMBODIMENT_CREATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/math/radians.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/cosm.hpp"
#include "cosm/pal/argos/cube_block_embodiment.hpp"
#include "cosm/pal/argos/ramp_block_embodiment.hpp"
#include "cosm/pal/argos/block_embodiment_variant.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::pal::argos {
class embodied_cube_block;
class embodied_ramp_block;
} /* namespace cosm::pal */

NS_START(cosm, pal, argos);

class swarm_manager_adaptor;

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct block_embodiment_creator
 * \ingroup pal argos
 *
 * \brief Action class for taking a 3D block of a given type and creating an
 * embodied representation within ARGoS for it.
 */
class block_embodiment_creator {
 public:
  block_embodiment_creator(const rmath::radians& z_rotation,
                           const rtypes::type_uuid& parent_id,
                           cpargos::swarm_manager_adaptor* sm)
      : mc_z_rot(z_rotation),
        mc_parent_id(parent_id),
        m_sm(sm) {}

  /* Not copy constructable/assignable by default */
  block_embodiment_creator(const block_embodiment_creator&) = delete;
  const block_embodiment_creator& operator=(const block_embodiment_creator&) = delete;

  cpargos::block_embodiment_variant operator()(
      const cpargos::embodied_cube_block* block) const;
  cpargos::block_embodiment_variant operator()(
      const cpargos::embodied_ramp_block* block) const;

 private:
  /**
   * \brief How thick to make each of the boxes used to approximate the
   * embodiment of a ramp block.
   */
  static constexpr const double kRAMP_BOX_THICKNESS = 0.0001;

  /**
   * \brief Generate the ARGoS box for the bottom of the ramp.
   *
   * \note The returned pointer is OWNING, despite being raw, because that's the
   * ARGoS API.
   */
  ::argos::CBoxEntity* ramp_bottom(const cpargos::embodied_ramp_block* block,
                                   const rmath::radians& z_rotation,
                                   const rtypes::type_uuid& parent_id) const;

  /**
   * \brief Generate the ARGoS box for the back of the ramp.
   *
   * \note The returned pointer is OWNING, despite being raw, because that's the
   * ARGoS API.
   */
  ::argos::CBoxEntity* ramp_back(const cpargos::embodied_ramp_block* block,
                                 const rmath::radians& z_rotation,
                                 const rtypes::type_uuid& parent_id) const;

  /**
   * \brief Generate the ARGoS box for the top (slope part) of the ramp.
   *
   * \note The returned pointer is OWNING, despite being raw, because that's the
   * ARGoS API.
   */
  ::argos::CBoxEntity* ramp_top(const cpargos::embodied_ramp_block* block,
                                const rmath::radians& z_rotation,
                                const rtypes::type_uuid& parent_id) const;

  /* clang-format off */
  const rmath::radians    mc_z_rot;
  const rtypes::type_uuid mc_parent_id;

  cpargos::swarm_manager_adaptor*    m_sm;
  /* clang-format off */
};

NS_END(argos, pal, cosm);

#endif /* INCLUDE_COSM_PAL_ARGOS_BLOCK_EMBODIMENT_CREATOR_HPP_ */
