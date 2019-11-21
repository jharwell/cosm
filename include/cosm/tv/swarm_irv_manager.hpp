/**
 * \file swarm_irv_manager.hpp
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

#ifndef INCLUDE_COSM_TV_SWARM_IRV_MANAGER_HPP_
#define INCLUDE_COSM_TV_SWARM_IRV_MANAGER_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <boost/optional.hpp>
#include <map>

#include "rcppsw/ds/type_map.hpp"
#include "rcppsw/er/client.hpp"

#include "cosm/tv/switchable_tv_generator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, tv);

namespace config {
struct swarm_irv_manager_config;
}

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class swarm_irv_manager
 * \ingroup tv
 *
 * \brief Internal Robot Variance (IRV) manager. Orchestrates all application of
 * temporal variance all robots in the swarm as it relates to their internal
 * state (no environmental/inter-robot variances).
 *
 * This class does not apply variances to each robot, as that possibly requires
 * platform-specific knowledge. Instead it houses the data for all possible
 * types of IRV, and specifies the interface for application. Currently, that
 * is:
 *
 * - Motion variances via throttling
 *
 * Each type of variance also has a corresponding pure virtual function
 * specifying how information about the state of the variance at the swarm level
 * can be extracted.
 */
class swarm_irv_manager : public rer::client<swarm_irv_manager> {
 public:
  explicit swarm_irv_manager(const config::swarm_irv_manager_config* config);

  swarm_irv_manager(const swarm_irv_manager& other) = delete;
  const swarm_irv_manager& operator=(const swarm_irv_manager& other) = delete;

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
    return (mc_motion_throttle_config) ? true : false;
  }

  const ctv::switchable_tv_generator* motion_throttling_handler(
      int robot_id) const {
    return &m_motion_throttling.at(robot_id);
  }

  /**
   * \brief Register a robot controller to the temporal variance controller so
   * that all necessary handlers for all possible types of variance that could
   * be applied to a given controller are setup.
   *
   * \param robot_id The ID of the robot controller, assumed to be unique (not
   * checked).
   */
  void register_controller(int robot_id);

  /* clang-format off */
  boost::optional<rct::config::waveform_config> mc_motion_throttle_config{};
  std::map<int, ctv::switchable_tv_generator>   m_motion_throttling{};
  /* clang-format on */
};

NS_END(tv, cosm);

#endif /* INCLUDE_COSM_TV_SWARM_IRV_MANAGER_HPP_ */
