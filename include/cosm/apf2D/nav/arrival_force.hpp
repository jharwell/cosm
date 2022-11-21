/**
 * \file arrival_force.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/er/client.hpp"
#include "rcppsw/rcppsw.hpp"

#include "cosm/cosm.hpp"
#include "cosm/apf2D/boid.hpp"
#include "cosm/apf2D/base_force.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav {

namespace config {
struct arrival_force_config;
} /* namespace config */

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class arrival_force
 * \ingroup apf2D nav
 *
 * \brief A force pulling the robot to a target (i.e. the robot DOES slow down
 * to "arrive"). Once the robot comes within range of the slowing radius, its
 * speed is ramped down linearly from its current speed to the specified minimal
 * slowing speed.
 */
class arrival_force : public rer::client<arrival_force>,
                      public capf2D::base_force {
 public:
  explicit arrival_force(const config::arrival_force_config* config);

  rmath::vector2d operator()(const boid& entity, const rmath::vector2d& target);
  bool within_slowing_radius(void) const { return m_within_slowing_radius; }

 private:
  /* clang-format off */
  const double mc_max;
  const double mc_slowing_speed_min;
  const double mc_slowing_radius;

  bool         m_within_slowing_radius{false};
  /* clang-format on */
};

} /* namespace cosm::apf2D::nav */
