/**
 * \file robot_dynamics_applicator.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>
#include <map>

#include "rcppsw/ds/type_map.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/types/type_uuid.hpp"

#include "cosm/tv/switchable_tv_generator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, tv);

namespace config {
struct robot_dynamics_applicator_config;
}

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class robot_dynamics_applicator
 * \ingroup tv
 *
 * \brief Orchestrates all application of all dynamics/temporal variance to all
 * robots in the swarm. This class does not apply variances to each robot, as
 * that possibly requires platform-specific knowledge. Instead it houses the
 * data for all possible types of variance which cannot be applied from inside
 * the robot (such as robot slowdown due to environmental conditions, object
 * weight, etc.), and specifies the interface for application, which must be
 * implemented by derived classes.
 *
 * Types of internal robot variance this manager handles are:
 *
 * - Motion variances via throttling
 *
 * This class maintains a map of <id, variance applicator> for each type of
 * variance for each robot, because not all robots experience identical
 * variances (eg variances might only be applied when they are carrying a
 * block).
 *
 * Each type of variance also has a corresponding pure virtual function
 * specifying how information about the state of the variance at the swarm level
 * can be extracted.
 */
class RCPPSW_EXPORT robot_dynamics_applicator : public rer::client<robot_dynamics_applicator> {
 public:
  explicit robot_dynamics_applicator(
      const config::robot_dynamics_applicator_config* config);

  robot_dynamics_applicator(const robot_dynamics_applicator&) = delete;
  const robot_dynamics_applicator&
  operator=(const robot_dynamics_applicator&) = delete;

  /**
   * \brief Update the state of all applied variances. Should be called once per
   * timestep.
   */
  virtual void update(void) = 0;

  /**
   * \brief Compute the average motion throttle that is currently being applied
   * to the swarm.
   */
  virtual double avg_motion_throttle(void) const = 0;

  bool motion_throttling_enabled(void) const {
    return (m_motion_throttle_config) ? true : false;
  }
  bool bc_throttling_enabled(void) const {
    return (m_bc_throttle_config) ? true : false;
  }

  RCPPSW_PURE const ctv::switchable_tv_generator*
  motion_throttler(const rtypes::type_uuid& id) const {
    return const_cast<robot_dynamics_applicator*>(this)->motion_throttler(id);
  }

  RCPPSW_PURE const ctv::switchable_tv_generator*
  bc_throttler(const rtypes::type_uuid& id) const {
    return const_cast<robot_dynamics_applicator*>(this)->bc_throttler(id);
  }

  /**
   * \brief Register a robot controller so that all necessary handlers for all
   * possible types of variance that could be applied to a given controller are
   * setup.
   *
   * \param id The ID of the robot controller.
   */
  void register_controller(const rtypes::type_uuid& id);

  /**
   * \brief Un-register a robot controller if it has been permanently removed
   * from simulation.
   *
   * \param id The ID of the robot controller.
   */
  void unregister_controller(const rtypes::type_uuid& id);

 protected:
  /**
   * \brief Get a reference to the motion throttler for a specific controller.
   */
  RCPPSW_PURE ctv::switchable_tv_generator*
  motion_throttler(const rtypes::type_uuid& id) {
    auto it = m_motion_throttlers.find(id);
    if (m_motion_throttlers.end() == it) {
      return nullptr;
    }
    return &it->second;
  }

  RCPPSW_PURE ctv::switchable_tv_generator*
  bc_throttler(const rtypes::type_uuid& id) {
    auto it = m_bc_throttlers.find(id);
    if (m_bc_throttlers.end() == it) {
      return nullptr;
    }
    return &it->second;
  }

 private:
  /* clang-format off */
  boost::optional<rct::config::waveform_config> m_motion_throttle_config{};
  boost::optional<rct::config::waveform_config> m_bc_throttle_config{};

  std::map<rtypes::type_uuid,
           ctv::switchable_tv_generator>        m_motion_throttlers{};
  std::map<rtypes::type_uuid,
           ctv::switchable_tv_generator>        m_bc_throttlers{};
  /* clang-format on */
};

NS_END(tv, cosm);

