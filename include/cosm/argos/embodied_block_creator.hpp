/**
 * \file embodied_block_creator.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_ARGOS_EMBODIED_BLOCK_CREATOR_HPP_
#define INCLUDE_COSM_ARGOS_EMBODIED_BLOCK_CREATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/radians.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/cosm.hpp"
#include "cosm/argos/embodied_cube_block.hpp"
#include "cosm/argos/embodied_ramp_block.hpp"
#include "cosm/argos/embodied_block_variant.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::repr {
class cube_block3D;
class ramp_block3D;
} /* namespace cosm::repr */

namespace cosm::pal::argos {
class swarm_manager_adaptor;
} /* namespace cosm::pal::argos */

NS_START(cosm, argos);


/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct embodied_block_creator
 * \ingroup argos
 *
 * \brief Action class for taking a 3D block of a given type and creating an
 * embodied representation within ARGoS for it.
 */
class embodied_block_creator {
 public:
  explicit embodied_block_creator(cpargos::swarm_manager_adaptor* sm) : m_sm(sm) {}

  cargos::embodied_block_varianto operator()(
      const crepr::cube_block3D* block) const;
  cargos::embodied_block_varianto operator()(
      const crepr::ramp_block3D* block) const;

 private:
  /* clang-format off */
  cpargos::swarm_manager_adaptor* m_sm;
  /* clang-format on */
};

NS_END(argos, cosm);

#endif /* INCLUDE_COSM_ARGOS_EMBODIED_BLOCK_CREATOR_HPP_ */
