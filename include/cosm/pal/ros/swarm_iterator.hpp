/**
 * \file swarm_iterator.hpp
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

#ifndef INCLUDE_COSM_PAL_ROS_SWARM_ITERATOR_HPP_
#define INCLUDE_COSM_PAL_ROS_SWARM_ITERATOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "cosm/pal/ros/swarm_manager_adaptor.hpp"
#include "cosm/hal/robot.hpp"
#include "cosm/ros/topic.hpp"
#include "cosm/pal/pal.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, pal, ros);

/*******************************************************************************
 * Struct Definitions
 ******************************************************************************/
/**
 * \struct swarm_iterator
 * \ingroup pal ros
 *
 * \brief Iterate over a swarm of robots running ROS. Iteration is always
 * performed in a static order (single threaded).
 */
struct swarm_iterator {
  /**
   * \brief Iterate through controllers using static ordering.
   *
   * \tparam TFunction Type of the lambda callback to use (inferred).
   *
   * \param sm Handle to the \ref cpros::swarm_manager_adaptor.
   * \param cb Function to run on each robot in the swarm.
   */
  template <typename TFunction>
  static void robots(size_t n_robots,
                     const TFunction& cb) {
    for (size_t i = 0; i < n_robots; ++i) {
      auto robot_ns = cros::topic("/") /
                      cpal::kRobotNamePrefix /
                      std::to_string(i);
      cb(robot_ns);
    } /* for(i..) */
  }
};

NS_END(ros, pal, cosm);

#endif /* INCLUDE_COSM_PAL_ROS_SWARM_ITERATOR_HPP_ */
