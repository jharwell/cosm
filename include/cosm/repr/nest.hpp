/**
 * \file nest.hpp
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

#ifndef INCLUDE_COSM_REPR_NEST_HPP_
#define INCLUDE_COSM_REPR_NEST_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>

#include <argos3/plugins/simulator/entities/light_entity.h>

#include "rcppsw/types/discretize_ratio.hpp"

#include "cosm/repr/colored_entity.hpp"
#include "cosm/repr/unicell_immovable_entity2D.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, repr);

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
             public repr::colored_entity {
 public:
  /**
   * \brief We use raw pointers to indicate we (COSM) do not own the
   * constructed lights. If we own them, then when ARGoS goes to delete them
   * after the experiment has ended the arena has already been deconstructed and
   * the nest lights along with them, and an exception is thrown.
   */
  using light_list = std::list<argos::CLightEntity*>;

  /**
   * \param dim Dimensions of the nest. Square nests get 1 light above the
   *            center while rectangular nests get 3 lights evenly spaced along
   *            the longer dimension.
   * \param loc The location of the center of the nest.
   * \param resolution The arena resolution used to map from continous sizes to
   *                   a discrete grid.
   * \param light_color The color to make the lights above the nest.
   */
  nest(const rmath::vector2d& dim,
       const rmath::vector2d& loc,
       const rtypes::discretize_ratio& resolution,
       const rutils::color& light_color);

  /**
   * \brief Determine if a real-valued point lies within the extent of the
   * nest for:
   *
   * 1. Visualization purposes.
   * 2. Determining if a robot has entered the nest.
   *
   * \param point The point to check.
   *
   * \return \c TRUE if the condition is met, and \c FALSE otherwise.
   */
  bool contains_point(const rmath::vector2d& point) const {
    return xspan().contains(point.x()) && yspan().contains(point.y());
  }

  light_list& lights(void) { return m_lights; }

 private:
  /**
   * \brief Initialize lights above the nest for robots to use for localization,
   * dependent on the geometry of the nest.
   */
  light_list init_lights(const rutils::color& color) const;
  light_list init_square(const rutils::color& color) const;
  light_list init_rect(const rutils::color& color) const;

  /* clang-format off */
  light_list m_lights;
  /* clang-format on */
};

NS_END(repr, cosm);

#endif /* INCLUDE_COSM_REPR_NEST_HPP_ */
