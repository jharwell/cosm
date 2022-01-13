/**
 * \file nest_light.hpp
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

#ifndef INCLUDE_COSM_REPR_NEST_LIGHT_HPP_
#define INCLUDE_COSM_REPR_NEST_LIGHT_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <argos3/plugins/simulator/entities/light_entity.h>
#include <string>

#include "rcppsw/types/spatial_dist.hpp"
#include "rcppsw/types/type_uuid.hpp"
#include "rcppsw/math/vector3.hpp"

#include "cosm/cosm.hpp"
#include "cosm/repr/colored_entity.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::pal::argos {
class swarm_manager_adaptor;
} /* namespace cosm::pal */

NS_START(cosm, repr);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class nest_light
 * \ingroup repr
 *
 * \brief Class representing a light that is associated with a \ref nest in the
 * arena.
 *
 * Lights must be sufficiently high above the ground and intense so that
 * robots can detect them from far away in large arenas, but not so bright and
 * intense as to saturate robot light sensors.
 */
class nest_light final : public repr::colored_entity {
 public:
  nest_light(const std::string& name,
             const rmath::vector3d& loc,
             const rutils::color& color,
             double intensity);

  nest_light(const nest_light&) = default;

  /* Not move constructable/assignable or copy assignable by default */
  nest_light& operator=(const nest_light&) = delete;
  nest_light(nest_light&&) = delete;
  nest_light& operator=(nest_light&&) = delete;

  void initialize(cpargos::swarm_manager_adaptor* sm);

 private:
  /* clang-format off */
  /**
   * \brief We use raw pointers to indicate we (COSM) do not own the
   * constructed lights. If we own them, then when ARGoS goes to delete them
   * after the experiment has ended the arena has already been deconstructed and
   * the nest lights along with them, and an exception is thrown.
   */
  argos::CLightEntity* m_impl;
  /* clang-format on */
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_NEST_LIGHT_HPP_ */
