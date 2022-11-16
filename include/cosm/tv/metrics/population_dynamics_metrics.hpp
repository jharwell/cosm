/**
 * \file population_dynamics_metrics.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
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

