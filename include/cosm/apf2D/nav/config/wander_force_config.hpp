/**
 * \file wander_force_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"

#include "cosm/cosm.hpp"
#include "cosm/apf2D/nav/config/bias_angle_config.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::nav::config {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct wander_force_config
 * \ingroup apf2D nav config
 *
 * \brief Configuration for the wander force, as described in \todo ref here.
 */
struct wander_force_config final : public rconfig::base_config {
  bias_angle_config bias_angle{};

  /**
   * \brief How often to apply the wander force. 1 = apply every time it is
   * asked. > 1 only apply every nth time it is asked. Depending on the
   * kinematics of the entity in question, applying the wander force every time
   * may not produce the desired level of exploration, because +/- random
   * perturbations to the velocity vector will, by definition, add up to 0 in
   * the long run. Depends on the angle generation used.
   */
  size_t interval{0};

  /**
   * \brief Maximum value of the wander force.
   */
  double max{0.0};

  /**
   * \brief Distance of the center of the circle used to calculate the wander
   * force from the entity. Larger value = higher magnitude of wander force
   * vector.
   */
  double circle_distance{0};

  /**
   * \brief Radius of the center of the circle used to calculate the wander
   * force. Large value = higher magnitude of wander force vector.
   */
  double circle_radius{0};
};

} /* namespace cosm::apf2D::nav::config */
