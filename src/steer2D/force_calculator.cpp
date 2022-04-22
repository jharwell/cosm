/**
 * \file force_calculator.cpp
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

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/steer2D/force_calculator.hpp"

#include "cosm/steer2D/config/force_calculator_config.hpp"
#include "cosm/steer2D/ds/path_state.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, steer2D);

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
force_calculator::force_calculator(boid& entity,
                                   const config::force_calculator_config* config)
    : ER_CLIENT_INIT("cosm.steer2D.force_calculator"),
      m_entity(entity),
      m_avoidance(&config->avoidance),
      m_arrival(&config->arrival),
      m_wander(&config->wander),
      m_polar(&config->polar),
      m_phototaxis(&config->phototaxis),
      m_path_following(&config->path_following) {}

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::vector2d force_calculator::seek_to(const rmath::vector2d& target) {
  rmath::vector2d force = m_arrival(m_entity, target);
  m_tracker.force_add("arrival", rutils::color::kBLUE, force); /* accum */

  ER_DEBUG("Arrival force: %s@%s [%f]",
           force.to_str().c_str(),
           force.angle().to_str().c_str(),
           force.length());
  return force;
} /* seek_to() */

rmath::vector2d force_calculator::wander(rmath::rng* rng) {
  rmath::vector2d force = m_wander(m_entity, rng);
  m_tracker.force_add("wander", rutils::color::kMAGENTA, force); /* accum */

  ER_DEBUG("Wander force: %s@%s [%f]",
           rcppsw::to_string(force).c_str(),
           rcppsw::to_string(force.angle()).c_str(),
           force.length());
  return force;
} /* wander() */

rmath::vector2d
force_calculator::avoidance(const rmath::vector2d& closest_obstacle) {
  rmath::vector2d force = m_avoidance(m_entity, closest_obstacle);
  m_tracker.force_add("avoidance", rutils::color::kRED, force); /* accum */

  ER_DEBUG("Avoidance force: %s@%s [%f]",
           rcppsw::to_string(force).c_str(),
           rcppsw::to_string(force.angle()).c_str(),
           force.length());
  return force;
} /* avoidance() */

rmath::vector2d force_calculator::phototaxis(
    const phototaxis_force::light_sensor_readings& readings) {
  rmath::vector2d force = m_phototaxis(readings);
  m_tracker.force_add(
      "phototaxis_light", rutils::color::kYELLOW, force); /* accum */

  ER_DEBUG("Phototaxis force: %s@%s [%f]",
           force.to_str().c_str(),
           force.angle().to_str().c_str(),
           force.length());
  return force;
} /* phototaxis() */

rmath::vector2d force_calculator::phototaxis(
    const phototaxis_force::camera_sensor_readings& readings,
    const rutils::color& color) {
  rmath::vector2d force = m_phototaxis(readings, color);
  m_tracker.force_add("phototaxis_camera", color, force); /* accum */

  ER_DEBUG("Phototaxis force: %s@%s [%f]",
           force.to_str().c_str(),
           force.angle().to_str().c_str(),
           force.length());
  return force;
} /* phototaxis() */

rmath::vector2d force_calculator::anti_phototaxis(
    const phototaxis_force::light_sensor_readings& readings) {
  rmath::vector2d force = -m_phototaxis(readings);
  m_tracker.force_add(
      "anti_phototaxis_light", rutils::color::kBLACK, force); /* accum */

  ER_DEBUG("Anti-phototaxis force: %s@%s [%f]",
           force.to_str().c_str(),
           force.angle().to_str().c_str(),
           force.length());
  return force;
} /* anti_phototaxis() */

rmath::vector2d force_calculator::anti_phototaxis(
    const phototaxis_force::camera_sensor_readings& readings,
    const rutils::color& color) {
  rmath::vector2d force = -m_phototaxis(readings, color);
  m_tracker.force_add("anti_phototaxis_camera", color, force); /* accum */

  ER_DEBUG("Anti-phototaxis force: %s@%s [%f]",
           force.to_str().c_str(),
           force.angle().to_str().c_str(),
           force.length());
  return force;
} /* anti_phototaxis() */

rmath::vector2d force_calculator::path_following(ds::path_state* state) {
  rmath::vector2d force = m_path_following(m_entity, state);
  m_tracker.path_add(*state); /* idempotent */
  m_tracker.force_add(
      "path_following", rutils::color::kORANGE, force); /* accum */

  ER_DEBUG("Path following force: %s@%s [%f]",
           force.to_str().c_str(),
           force.angle().to_str().c_str(),
           force.length());
  return force;
} /* path_following() */

rmath::vector2d force_calculator::polar(const rmath::vector2d& center) {
  rmath::vector2d force = m_polar(m_entity, center);
  m_tracker.force_add("polar", rutils::color::kCYAN, force); /* accum */

  ER_DEBUG("Polar force: %s@%s [%f]",
           force.to_str().c_str(),
           force.angle().to_str().c_str(),
           force.length());
  return force;
} /* polar() */

void force_calculator::forces_reset(void) {
  m_force_accum.set(0, 0);
} /* forces_reset() */

void force_calculator::tracking_reset(void) {
  m_tracker.reset();
} /* tracking_reset() */

NS_END(steer2D, cosm);
