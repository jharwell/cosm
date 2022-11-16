/**
 * \file path_following_force.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/rcppsw.hpp"

#include "cosm/cosm.hpp"
#include "cosm/steer2D/boid.hpp"
#include "cosm/steer2D/seek_force.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::steer2D::config {
struct path_following_force_config;
} /* namespace cosm::steer2D::config */

namespace cosm::steer2D::ds {
class path_state;
} /* namespace cosm::steer2D::ds */

NS_START(cosm, steer2D);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class path_following_force
 * \ingroup steer2D
 *
 * \brief A force pushing the robot along a trajectory defined by a set of
 * points in 2D.
 */
class path_following_force {
 public:
  explicit path_following_force(const config::path_following_force_config* config);

  /**
   * \brief Calculate the path following force that should be applied to the
   * robot. The force will point from the robot towards the next point along the
   * path. Once the robot has finished executing the path, the returned force
   * will be 0.
   *
   * \param entity The robot to calculate the force for.
   * \param state The current path state.
   */
  rmath::vector2d operator()(const boid& entity,
                             csteer2D::ds::path_state* state) const;

 private:
  /* clang-format off */
  const double mc_max;
  const double mc_radius;

  seek_force   m_seek;
  /* clang-format on */
};

NS_END(steer2D, cosm);
