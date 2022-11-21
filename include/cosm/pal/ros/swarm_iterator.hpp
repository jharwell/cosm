/**
 * \file swarm_iterator.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

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
namespace cosm::pal::ros {

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
      auto robot_ns = cros::to_ns(rtypes::type_uuid(i));
      cb(robot_ns);
    } /* for(i..) */
  }
};

} /* namespace cosm::pal::ros */
