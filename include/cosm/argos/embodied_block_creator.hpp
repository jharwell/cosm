/**
 * \file embodied_block_creator.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/radians.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/argos/embodied_block_variant.hpp"
#include "cosm/argos/embodied_cube_block.hpp"
#include "cosm/argos/embodied_ramp_block.hpp"
#include "cosm/cosm.hpp"

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
  explicit embodied_block_creator(cpargos::swarm_manager_adaptor* sm)
      : m_sm(sm) {}

  cargos::embodied_block_varianto
  operator()(const crepr::cube_block3D* block) const;
  cargos::embodied_block_varianto
  operator()(const crepr::ramp_block3D* block) const;

 private:
  /* clang-format off */
  cpargos::swarm_manager_adaptor* m_sm;
  /* clang-format on */
};

NS_END(argos, cosm);
