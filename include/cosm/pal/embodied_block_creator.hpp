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

#ifndef INCLUDE_COSM_PAL_EMBODIED_BLOCK_CREATOR_HPP_
#define INCLUDE_COSM_PAL_EMBODIED_BLOCK_CREATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/radians.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/cosm.hpp"
#include "cosm/pal/embodied_cube_block.hpp"
#include "cosm/pal/embodied_ramp_block.hpp"
#include "cosm/pal/embodied_block_variant.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::repr {
class cube_block3D;
class ramp_block3D;
} /* namespace cosm::repr */

NS_START(cosm, pal);

class argos_sm_adaptor;

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct embodied_block_creator
 * \ingroup pal
 *
 * \brief Action class for taking a 3D block of a given type and creating an
 * embodied representation within ARGoS for it.
 */
class embodied_block_creator {
 public:
  explicit embodied_block_creator(cpal::argos_sm_adaptor* sm) : m_sm(sm) {}

  cpal::embodied_block_varianto operator()(const crepr::cube_block3D* block) const;
  cpal::embodied_block_varianto operator()(const crepr::ramp_block3D* block) const;

 private:
  /* clang-format off */
  cpal::argos_sm_adaptor* m_sm;
  /* clang-format on */
};

NS_END(pal, cosm);

#endif /* INCLUDE_COSM_PAL_EMBODIED_BLOCK_CREATOR_HPP_ */
