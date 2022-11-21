/**
 * \file constant_speed_force_config.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"

#include "rcppsw/math/radians.hpp"
#include "rcppsw/spatial/euclidean_dist.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::apf2D::flocking::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct constant_speed_force_config
 * \ingroup spatial strategy flocking config
 *
 * \brief Configuration for \ref capf2D::constant_speed_force.
 */
struct constant_speed_force_config : public rconfig::base_config {
  /**
   * \brief The maximum strength of the force.
   */
  double max{0};

  /**
   * \brief The "critical" speed the the swarm is collectively trying to
   *        maintain.
   *
   * \f$v_c\f$ in \cite FLOCK:Bagarti2018-stochfov.
   */

  double speed_setpoint{0};
};


} /* namespace cosm::apf2D::flocking::config */
