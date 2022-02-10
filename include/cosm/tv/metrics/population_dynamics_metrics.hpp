/**
 * \file population_dynamics_metrics.hpp
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
 * \class population_dynamics_metrics
 * \ingroup metrics tv
 *
 * \brief Defines the metrics to be collected from \ref population_dynamics
 * about the different types of population dynamics that can be applied to the
 * swarm.
 *
 * Metrics are collected and output every timestep.
 */
class population_dynamics_metrics : public virtual rmetrics::base_metrics {
 public:
  struct queue_op_status {
    size_t count;
    rtypes::timestep interval_accum;
  };
  struct queue_status {
    size_t size;
    double lambda;
    double mu;
    struct queue_op_status enqueue;
    struct queue_op_status dequeue;
  };
  population_dynamics_metrics(void) = default;

  virtual queue_status death_queue_status(void) const = 0;
  virtual queue_status birth_queue_status(void) const = 0;
  virtual queue_status repair_queue_status(void) const = 0;

  /**
   * \brief Return the total number of robots in the arena, regardless of
   * whether they are active or not (i.e. including ones being repaired).
   */
  virtual size_t swarm_total_population(void) const = 0;

  /**
   * \brief Return the total number of robots in the arena that are active
   * (i.e. in the arena and NOT being repaired).
   */
  virtual size_t swarm_active_population(void) const = 0;

  /**
   * \brief Return the maximum size of the swarm, which is assumed to be
   * constant.
   */
  virtual size_t swarm_max_population(void) const = 0;
};

NS_END(metrics, tv, cosm);

