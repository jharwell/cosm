/**
 * \file env_dynamics_metrics.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_TV_ENV_DYNAMICS_METRICS_HPP_
#define INCLUDE_COSM_TV_ENV_DYNAMICS_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, tv, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class base_env_dynamics_metrics
 * \ingroup tv metrics
 *
 * \brief Defines the metrics to be collected from the environment and the swarm
 * about the different types of environmental variance that can be applied to
 * each. Project-specific types of environmental variances (e.g. those relating
 * to caches can be defined by deriving from this class).
 *
 * Not really "metrics" per-se, but more of a way to record variances for later
 * usage in post-processing.
 *
 * Metrics are collected and output EVERY timestep.
 */
class base_env_dynamics_metrics : public virtual rmetrics::base_metrics {
 public:
  base_env_dynamics_metrics(void) = default;
  virtual ~base_env_dynamics_metrics(void) = default;

  /**
   * \brief Return the average motion throttling within the swarm, as a
   * percentage [0, 1.0].
   */
  virtual double avg_motion_throttle(void) const = 0;

  /**
   * \brief Return the current value of the block manipulation penalty present
   * in the arena.
   */
  virtual rtypes::timestep arena_block_manip_penalty(void) const = 0;
};

NS_END(metrics, tv, cosm);

#endif /* INCLUDE_COSM_TV_ENV_DYNAMICS_METRICS_HPP_ */
