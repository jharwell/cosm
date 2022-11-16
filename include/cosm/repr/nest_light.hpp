/**
 * \file nest_light.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include <argos3/plugins/simulator/entities/light_entity.h>

#include "rcppsw/math/vector3.hpp"
#include "rcppsw/spatial/euclidean_dist.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/cosm.hpp"
#include "cosm/repr/colored_entity.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::pal::argos {
class swarm_manager_adaptor;
} // namespace cosm::pal::argos

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
  ::argos::CLightEntity* m_impl;
  /* clang-format on */
};

NS_END(repr, cosm);
