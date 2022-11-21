/**
 * \file apf_manager.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>
#include <vector>

#include "rcppsw/er/client.hpp"
#include "rcppsw/rcppsw.hpp"
#include "rcppsw/math/rng.hpp"

#include "cosm/apf2D/tracker.hpp"
#include "cosm/apf2D/boid.hpp"
#include "cosm/apf2D/nav/phototaxis_force.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D {

namespace nav {
class arrival_force;
class avoidance_force;
class path_following_force;
class polar_force;
class wander_force;
} /* namespace nav */

namespace flocking {
class constant_speed_force;
class alignment_force;
} /* namespace flocking */

namespace config {
struct apf_manager_config;
} /* namespace config */

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class apf_manager
 * \ingroup apf2D
 *
 * \brief Class encapsulating steering of entities through 2D space via summing
 * selectable forces that act on the entity each timestep (artificial potential
 * fields). To use this class, entities must conform to the \ref boid interface.
 */
class apf_manager : public rer::client<apf_manager> {
 public:
  apf_manager(boid& entity, const config::apf_manager_config* config);

  ~apf_manager(void);

  /**
   * \brief Return the current net APF force as a velocity vector.
   */
  const rmath::vector2d& value(void) const { return m_force_accum; }
  void value(const rmath::vector2d& val) { m_force_accum = val; }

  const class tracker* tracker(void) const { return &m_tracker; }
  class tracker* tracker(void) {
    return &m_tracker;
  }

  /**
   * \brief Reset the sum of forces acting on the entity.
   */
  void forces_reset(void);

  /**
   * \brief Reset the force/path tracking.
   */
  void tracking_reset(void);

  bool is_enabled(void) const { return m_enabled; }
  void enable(void) { m_enabled = true; }
  void disable(void) { m_enabled = false; }

  /**
   * \brief Calculate the \ref nav::arrival_force for this timestep.
   *
   * \param target The target to seek to.
   */
  rmath::vector2d seek_to(const rmath::vector2d& target);

  /**
   * \brief Calculate the \ref nav::wander_force for this timestep.
   */
  rmath::vector2d wander(rmath::rng* rng);

  /**
   * \brief Calculate the \ref nav::avoidance_force for this timestep.
   *
   * If no threatening obstacle exists, this force is 0.
   *
   * \param closest_obstacle Where is the closest obstacle, relative to robot's
   * current position AND heading.
   */
  rmath::vector2d avoidance(const rmath::vector2d& closest_obstacle);

  /**
   * \brief Calculate the \ref nav::phototaxis_force for this timestep.
   *
   * \param readings The current light sensor readings.
   */
  rmath::vector2d
  phototaxis(const nav::phototaxis_force::light_sensor_readings& readings);

  /**
   * \brief Calculate the \ref nav::phototaxis_force for this timestep.
   *
   * \param readings The current camera sensor readings.
   * \param color The color of the light source to taxis towards. If any of the
   *              camera sensor readings are not this color, they are ignored in
   *              the force calculation.
   */
  rmath::vector2d
  phototaxis(const nav::phototaxis_force::camera_sensor_readings& readings,
             const rutils::color& color);

  /**
   * \brief Calculate the \ref nav::path_following_force for this timestep.
   *
   * \param state The current path state.
   */
  rmath::vector2d path_following(nav::ds::path_state* state);

  /**
   * \brief Calculate the \ref nav::polar_force for this timestep.
   */
  rmath::vector2d polar(const rmath::vector2d& center);

  /**
   * \brief Calculate the negative of the \ref nav::phototaxis_force for this
   * timestep.
   *
   * \param readings The current light sensor readings.
   */
  rmath::vector2d
  anti_phototaxis(const nav::phototaxis_force::light_sensor_readings& readings);

  /**
   * \brief Calculate the negative of the \ref nav::phototaxis_force for this
   * timestep.
   *
   * \param readings The current camera sensor readings.
   * \param color The color of the light source to taxis towards. If any of the
   *              camera sensor readings are not this color, they are ignored in
   *              the force calculation.
   */
  rmath::vector2d
  anti_phototaxis(const nav::phototaxis_force::camera_sensor_readings& readings,
                  const rutils::color& color);

  /**
   * \brief Calculate the \ref flocking:alignment_force for this timestep.
   *
   * \param others The VELOCITIES of other agents to which the current agent
   *               should try to align its heading to the centroid of.
   */
  rmath::vector2d alignment(const std::vector<rmath::vector2d> others);

  /**
   * \brief Calculate the \ref flocking:constant_speed_force for this timestep.
   *
   * \param others The VELOCITIES of other agents to which the current agent
   *               should try to align its heading to the centroid of.
   */
  rmath::vector2d constant_speed(const std::vector<rmath::vector2d> others);

  /**
   * \brief Add a calculated force to the running total of the forces acting on
   * the agent since the last reset. Does not apply the force(s) to the agent.
   */
  void accum(const rmath::vector2d& force);

  bool within_slowing_radius(void) const;

 private:
  const boid& entity(void) const { return m_entity; }

  /* clang-format off */
  bool                                            m_enabled{true};
  boid&                                           m_entity;
  rmath::vector2d                                 m_force_accum{};
  std::unique_ptr<nav::avoidance_force>           m_avoidance;
  std::unique_ptr<nav::arrival_force>             m_arrival;
  std::unique_ptr<nav::wander_force>              m_wander;
  std::unique_ptr<nav::polar_force>               m_polar;
  std::unique_ptr<nav::phototaxis_force>          m_phototaxis;
  std::unique_ptr<nav::path_following_force>      m_path_following;

  std::unique_ptr<flocking::alignment_force>      m_alignment;
  std::unique_ptr<flocking::constant_speed_force> m_constant_speed;

  class tracker                                   m_tracker{};
  /* clang-format on */
};

} /* namespace cosm::apf2D */
