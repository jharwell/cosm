/**
 * \file apf_manager.cpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/apf2D/apf_manager.hpp"

#include "cosm/apf2D/config/apf_manager_config.hpp"
#include "cosm/apf2D/nav/ds/path_state.hpp"
#include "cosm/apf2D/nav/arrival_force.hpp"
#include "cosm/apf2D/nav/avoidance_force.hpp"
#include "cosm/apf2D/nav/path_following_force.hpp"
#include "cosm/apf2D/nav/phototaxis_force.hpp"
#include "cosm/apf2D/nav/polar_force.hpp"
#include "cosm/apf2D/nav/wander_force.hpp"
#include "cosm/apf2D/flocking/alignment_force.hpp"
#include "cosm/apf2D/flocking/constant_speed_force.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D {

/*******************************************************************************
 * Constructors/Destructors
 ******************************************************************************/
apf_manager::apf_manager(boid& entity,
                 const config::apf_manager_config* config)
    : ER_CLIENT_INIT("cosm.apf2D.apf_manager"),
      m_entity(entity),
      m_avoidance(std::make_unique<nav::avoidance_force>(&config->nav.avoidance)),
      m_arrival(std::make_unique<nav::arrival_force>(&config->nav.arrival)),
      m_wander(std::make_unique<nav::wander_force>(&config->nav.wander)),
      m_polar(std::make_unique<nav::polar_force>(&config->nav.polar)),
      m_phototaxis(std::make_unique<nav::phototaxis_force>(&config->nav.phototaxis)),
      m_path_following(std::make_unique<nav::path_following_force>(&config->nav.path_following)),
      m_alignment(std::make_unique<flocking::alignment_force>(&config->flocking.alignment)),
      m_constant_speed(std::make_unique<flocking::constant_speed_force>(&config->flocking.constant_speed)) {}

apf_manager::~apf_manager(void) = default;

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
rmath::vector2d apf_manager::seek_to(const rmath::vector2d& target) {
  rmath::vector2d force = m_arrival->operator()(m_entity, target);
  m_tracker.force_add("arrival", rutils::color::kBLUE, force); /* accum */

  ER_DEBUG("Arrival force: %s@%s [%f]",
           force.to_str().c_str(),
           force.angle().to_str().c_str(),
           force.length());
  return force;
} /* seek_to() */

rmath::vector2d apf_manager::wander(rmath::rng* rng) {
  rmath::vector2d force = m_wander->operator()(m_entity, rng);
  m_tracker.force_add("wander", rutils::color::kMAGENTA, force); /* accum */

  ER_DEBUG("Wander force: %s@%s [%f]",
           rcppsw::to_string(force).c_str(),
           rcppsw::to_string(force.angle()).c_str(),
           force.length());
  return force;
} /* wander() */

rmath::vector2d
apf_manager::avoidance(const rmath::vector2d& closest_obstacle) {
  rmath::vector2d force = m_avoidance->operator()(m_entity, closest_obstacle);
  m_tracker.force_add("avoidance", rutils::color::kRED, force); /* accum */

  ER_DEBUG("Avoidance force: %s@%s [%f]",
           rcppsw::to_string(force).c_str(),
           rcppsw::to_string(force.angle()).c_str(),
           force.length());
  return force;
} /* avoidance() */

rmath::vector2d apf_manager::phototaxis(
    const nav::phototaxis_force::light_sensor_readings& readings) {
  rmath::vector2d force = m_phototaxis->operator()(readings);
  m_tracker.force_add(
      "phototaxis_light", rutils::color::kYELLOW, force); /* accum */

  ER_DEBUG("Phototaxis force: %s@%s [%f]",
           force.to_str().c_str(),
           force.angle().to_str().c_str(),
           force.length());
  return force;
} /* phototaxis() */

rmath::vector2d apf_manager::phototaxis(
    const nav::phototaxis_force::camera_sensor_readings& readings,
    const rutils::color& color) {
  rmath::vector2d force = m_phototaxis->operator()(readings, color);
  m_tracker.force_add("phototaxis_camera", color, force); /* accum */

  ER_DEBUG("Phototaxis force: %s@%s [%f]",
           force.to_str().c_str(),
           force.angle().to_str().c_str(),
           force.length());
  return force;
} /* phototaxis() */

rmath::vector2d apf_manager::anti_phototaxis(
    const nav::phototaxis_force::light_sensor_readings& readings) {
  rmath::vector2d force = -m_phototaxis->operator()(readings);
  m_tracker.force_add(
      "anti_phototaxis_light", rutils::color::kBLACK, force); /* accum */

  ER_DEBUG("Anti-phototaxis force: %s@%s [%f]",
           force.to_str().c_str(),
           force.angle().to_str().c_str(),
           force.length());
  return force;
} /* anti_phototaxis() */

rmath::vector2d apf_manager::anti_phototaxis(
    const nav::phototaxis_force::camera_sensor_readings& readings,
    const rutils::color& color) {
  rmath::vector2d force = -m_phototaxis->operator()(readings, color);
  m_tracker.force_add("anti_phototaxis_camera", color, force); /* accum */

  ER_DEBUG("Anti-phototaxis force: %s@%s [%f]",
           force.to_str().c_str(),
           force.angle().to_str().c_str(),
           force.length());
  return force;
} /* anti_phototaxis() */

rmath::vector2d apf_manager::path_following(nav::ds::path_state* state) {
  rmath::vector2d force = m_path_following->operator()(m_entity, state);
  m_tracker.path_add(*state); /* idempotent */
  m_tracker.force_add(
      "path_following", rutils::color::kORANGE, force); /* accum */

  ER_DEBUG("Path following force: %s@%s [%f]",
           force.to_str().c_str(),
           force.angle().to_str().c_str(),
           force.length());
  return force;
} /* path_following() */

rmath::vector2d apf_manager::alignment(const std::vector<rmath::vector2d> others) {
  rmath::vector2d force = m_alignment->operator()(m_entity, others);
  m_tracker.force_add("alignment", rutils::color::kCYAN, force); /* accum */

  ER_DEBUG("Alignment force: %s@%s [%f]",
           force.to_str().c_str(),
           force.angle().to_str().c_str(),
           force.length());
  return force;
} /* alignment() */

rmath::vector2d apf_manager::constant_speed(const std::vector<rmath::vector2d> others) {
  rmath::vector2d force = m_constant_speed->operator()(m_entity, others);
  m_tracker.force_add("constant_speed", rutils::color::kCYAN, force); /* accum */

  ER_DEBUG("Constant speed force: %s@%s [%f]",
           force.to_str().c_str(),
           force.angle().to_str().c_str(),
           force.length());
  return force;
} /* constant_speed() */

rmath::vector2d apf_manager::polar(const rmath::vector2d& center) {
  rmath::vector2d force = m_polar->operator()(m_entity, center);
  m_tracker.force_add("polar", rutils::color::kCYAN, force); /* accum */

  ER_DEBUG("Polar force: %s@%s [%f]",
           force.to_str().c_str(),
           force.angle().to_str().c_str(),
           force.length());
  return force;
} /* polar() */

void apf_manager::accum(const rmath::vector2d& force) {
  ER_DEBUG("Current force: %s, new=%s",
           rcppsw::to_string(m_force_accum).c_str(),
           rcppsw::to_string(force).c_str())
  m_force_accum += force;
}

void apf_manager::forces_reset(void) {
  m_force_accum.set(0, 0);
} /* forces_reset() */

void apf_manager::tracking_reset(void) {
  m_tracker.reset();
} /* tracking_reset() */

bool apf_manager::within_slowing_radius(void) const {
  return m_arrival->within_slowing_radius();
} /* within_slowing_radius() */

} /* namespace cosm::apf2D */
