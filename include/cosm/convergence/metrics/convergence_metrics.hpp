/**
 * \file convergence_metrics.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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
#include <tuple>

#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/rcppsw.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, convergence, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class convergence_metrics
 * \ingroup convergence metrics
 *
 * \brief Defines the metrics to be collected at the ensemble level in order
 * to calculate the swarm's convergence in various ways. Measures adapted from:
 *
 * - Szabo2014 (robot interaction degree)
 * - Turgut2008 (angular order, velocity)
 * - Turgut2008/Balch2000 (positional entropy)
 *
 * Metrics are collected every timestep.
 */
class convergence_metrics : public virtual rmetrics::base_metrics {
 public:
  convergence_metrics(void) = default;
  /**
   * \brief Status returned by each convergence measure. Positions:
   *
   * 0 - Raw value of convergence measure, for sanity checks.
   * 1 - Normalized value of convergence measure.
   * 2 - Does the measure currently indicate convergence?
   */
  using conv_status_t = std::tuple<double, double, bool>;

  /**
   * \brief Return the current convergence epsilon (threshold) for the swarm
   * (can be static or vary in time). Captured here to make graph generation
   * much easier in SIERRA.
   */
  virtual double swarm_conv_epsilon(void) const = 0;

  /**
   * \brief Return the \ref conv_status_t for the interaction degree of the
   * swarm. See \ref interactivity for the calculation/input variables.
   */
  virtual conv_status_t swarm_interactivity(void) const = 0;

  /**
   * \brief Return the \ref conv_status_t for the angular order of the
   * swarm. See \ref angular_order for the calculation/input variables.
   */
  virtual conv_status_t swarm_angular_order(void) const = 0;

  /**
   * \brief Return the \ref conv_status_t for the positional entropy of the
   * swarm. See \ref positional_entropy for the calculation/input variables.
   */
  virtual conv_status_t swarm_positional_entropy(void) const = 0;

  /**
   * \brief Return the \ref conv_status_t for the task distribution entropy of
   * the swarm. See \ref task_dist_entropy for the calculation/input variables.
   */
  virtual conv_status_t swarm_task_dist_entropy(void) const = 0;

  /**
   * \brief Return the \ref conv_status_t for the velocity of the swarm. See
   * \ref velocity for the calculation/input variables.
   */
  virtual conv_status_t swarm_velocity(void) const = 0;
};

NS_END(metrics, convergence, cosm);

