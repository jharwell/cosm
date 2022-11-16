/**
 * \file stoch_fov_config.hpp
 *
 * \copyright 2022 SIFT LLC, All rights reserved.
 *
 * SPDX-License Identifier: LGPL-2.0-or-later
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
namespace cosm::spatial::strategy::flocking::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct stoch_fov_config
 * \ingroup spatial strategy flocking config
 *
 * \brief Configuration for \ref cssflocking::stoch_fov.
 */
struct stoch_fov_config : public rconfig::base_config {
  /**
   * \brief The strength of the interaction. \f$\alpha < 1\f$ in the paper.
   */
  double strength{-1};

  /**
   * \brief The "critical" speed the the swarm is collectively trying to
   *        maintain.
   *
   * \f$v_c\f$ in the paper.
   */

  double critical_speed{-1};

  /**
   * \brief The maximum bearing angle between two agents to consider for
   * interaction.
   *
   * \f$\theta_{max}\f$ in the paper.
   */
  rmath::radians theta_max{};

  /**
   * \brief The mean/"preferred" interaction distance.
   *
   * \f$\sigma\f$ in the paper.
   */
  rspatial::euclidean_dist mean_interaction_dist{0};
};


} /* namespace cosm::spatial::strategy::flocking::config */
