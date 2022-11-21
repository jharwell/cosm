/**
 * \file stoch_fov_config.hpp
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

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::flocking::config {

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct stoch_fov_config
 * \ingroup flocking config
 *
 * \brief Configuration for \ref cflocking::stoch_fov.
 */
struct stoch_fov_config : public rconfig::base_config {
  /**
   * \brief The maximum bearing angle between the agent's orientation and
   * something another agent to consider interaction.
   *
   * \f$\theta_{ij}\f$ in \cite FLOCK:Bagarti2018-stochfov.
   */
  rmath::radians theta_max{};

  /**
   * \brief The mean/"preferred" interaction distance.
   *
   * \f$\sigma\f$ in \cite FLOCK:Bagarti2018-stochfov.
   */
  rspatial::euclidean_dist mean_interaction_dist{0};
};


} /* namespace cosm::flocking::config */
