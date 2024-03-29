/**
 * \file nest.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <string>

#include "rcppsw/types/discretize_ratio.hpp"

#include "cosm/repr/config/nest_config.hpp"
#include "cosm/repr/nest_light.hpp"
#include "cosm/repr/unicell_immovable_entity2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::repr {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class nest
 * \ingroup repr
 *
 * \brief Class representing the arena nest in simulation, which is
 * multi-cellular and immobile. Obviously, it can only be used when compiling
 * COSM for ARGoS.
 *
 * When initializing lights, they will only be detectable by the robots's light
 * sensor, NOT the omnidirectional camera, as that can only detect LED entities
 * that are on the ground (implementation detail of ARGoS).
 */
class nest : public repr::unicell_immovable_entity2D,
             public repr::colored_entity,
             public rer::client<nest>,
             public rer::stringizable {
 public:
  nest(const config::nest_config* config,
       const rmath::vector2d& arena_dim,
       const rtypes::discretize_ratio& resolution);

  const std::list<nest_light>& lights(void) { return m_lights; }

  std::string to_str(void) const override;

  /**
   * \brief Initialize lights above the nest for robots to use for localization,
   * dependent on the geometry of the nest.
   *
   * \param light_color The color to make the lights above the nest.
   */
  void initialize(cpargos::swarm_manager_adaptor* sm,
                  const rutils::color& light_color);

 private:
  /**
   * The ID that will be assigned to the next nest created.
   */
  static int m_nest_id;

  static rspatial::euclidean_dist light_height_calc(const rmath::vector2d& arena) {
    return rspatial::euclidean_dist{ std::pow(arena.x() * arena.y(), 0.25) };
  }

  static double light_intensity_calc(const rmath::vector2d& arena) {
    return std::pow(arena.x() * arena.y(), 0.5);
  }

  std::list<nest_light> init_square(const rutils::color& color);
  std::list<nest_light> init_rect(const rutils::color& color);

  /* clang-format off */
  const double               mc_light_intensity;
  const rspatial::euclidean_dist mc_light_height;

  bool                       m_initialized{false};
  std::list<nest_light>      m_lights{};
  /* clang-format on */
};

} /* namespace cosm::repr */
