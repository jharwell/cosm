/**
 * @file force_calculator.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_STEER2D_FORCE_CALCULATOR_HPP_
#define INCLUDE_COSM_STEER2D_FORCE_CALCULATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/rcppsw.hpp"

#include "cosm/kin/twist.hpp"
#include "cosm/steer2D/arrival_force.hpp"
#include "cosm/steer2D/avoidance_force.hpp"
#include "cosm/steer2D/phototaxis_force.hpp"
#include "cosm/steer2D/polar_force.hpp"
#include "cosm/steer2D/seek_force.hpp"
#include "cosm/steer2D/wander_force.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D);
namespace config {
struct force_calculator_config;
} /* namespace config */

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class force_calculator
 * @ingroup cosm steer2D
 *
 * @brief Class encapsulating steering of entities through 2D space via summing
 * selectable forces that act on the entity each timestep. To use this class,
 * entities must conform to the \ref boid interface.
 */
class force_calculator : public rer::client<force_calculator> {
 public:
  force_calculator(boid& entity, const config::force_calculator_config* config);

  /**
   * @brief Return the current steering force as a velocity vector.
   */
  const rmath::vector2d& value(void) const { return m_force_accum; }
  void value(const rmath::vector2d& val) { m_force_accum = val; }

  /**
   * @brief Return the current steering force as twist acting on the managed
   * entity.
   */
  kin::twist value_as_twist(void) const { return to_twist(m_force_accum); }

  kin::twist to_twist(const rmath::vector2d& force) const;

  /**
   * @brief Reset the sum of forces acting on the entity.
   */
  void reset(void) { m_force_accum.set(0, 0); }

  /**
   * @brief Calculate the \ref arrival_force for this timestep.
   *
   * @param target The target to seek to.
   */
  rmath::vector2d seek_through(const rmath::vector2d& target);

  /**
   * @brief Calculate the \ref seek_force for this timestep.
   *
   * @param target The target to seek to.
   */
  rmath::vector2d seek_to(const rmath::vector2d& target);

  bool within_slowing_radius(void) const {
    return m_arrival.within_slowing_radius();
  }
  /**
   * @brief Calculate the \ref wander_force for this timestep.
   */
  rmath::vector2d wander(rmath::rng* rng);

  /**
   * @brief Calculate the \ref avoidance_force for this timestep.
   *
   * If no threatening obstacle exists, this force is 0.
   *
   * @param closest_obstacle Where is the closest obstacle, relative to robot's
   * current position AND heading.
   */
  rmath::vector2d avoidance(const rmath::vector2d& closest_obstacle);

  /**
   * @brief Calculate the \ref phototaxis_force for this timestep.
   *
   * @param readings The current light sensor readings.
   */
  rmath::vector2d phototaxis(
      const phototaxis_force::light_sensor_readings& readings);

  /**
   * @brief Calculate the \ref phototaxis_force for this timestep.
   *
   * @param readings The current camera sensor readings.
   * @param color The color of the light source to taxis towards. If any of the
   *              camera sensor readings are not this color, they are ignored in
   *              the force calculation.
   */
  rmath::vector2d phototaxis(
      const phototaxis_force::camera_sensor_readings& readings,
      const rutils::color& color);

  /**
   * @brief Calculate the negative of the \ref phototaxis_force for this
   * timestep.
   *
   * @param readings The current light sensor readings.
   */
  rmath::vector2d anti_phototaxis(
      const phototaxis_force::light_sensor_readings& readings);

  /**
   * @brief Calculate the negative of the \ref phototaxis_force for this
   * timestep.
   *
   * @param readings The current camera sensor readings.
   * @param color The color of the light source to taxis towards. If any of the
   *              camera sensor readings are not this color, they are ignored in
   *              the force calculation.
   */
  rmath::vector2d anti_phototaxis(
      const phototaxis_force::camera_sensor_readings& readings,
      const rutils::color& color);

  void accum(const rmath::vector2d& force) { m_force_accum += force; }

 private:
  const boid& entity(void) const { return m_entity; }

  /* clang-format off */
  boid&            m_entity;
  rmath::vector2d   m_force_accum{};
  avoidance_force  m_avoidance;
  arrival_force    m_arrival;
  seek_force       m_seek{};
  wander_force     m_wander;
  polar_force      m_polar;
  phototaxis_force m_phototaxis;
  /* clang-format on */
};

NS_END(steer2D, cosm);

#endif /* INCLUDE_COSM_STEER2D_FORCE_CALCULATOR_HPP_ */
