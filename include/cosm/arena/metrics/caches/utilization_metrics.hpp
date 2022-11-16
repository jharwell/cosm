/**
 * \file utilization_metrics.hpp
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
#include "rcppsw/rcppsw.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, arena, metrics, caches);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class utilization_metrics
 * \ingroup arena metrics caches
 *
 * \brief Interface defining utilization metrics that can be collected on \ref
 * arena_cache objects in the arena during their lifetime.
 */
class utilization_metrics : virtual public rmetrics::base_metrics {
 public:
  utilization_metrics(void) = default;
  ~utilization_metrics(void) override = default;
  utilization_metrics(const utilization_metrics&) = default;
  utilization_metrics& operator=(const utilization_metrics&) = default;

  /**
   * \brief Get the # of blocks currently in the cache (independent of any
   * calls to \ref reset_metrics()).
   */
  virtual size_t n_blocks(void) const = 0;

  /**
   * \brief Should return the # of blocks a given cache has had picked up from
   * it this timestep.
   *
   * This currently has a max of 1, due to limitations/shortcuts taken with the
   * block pickup events.
   */
  virtual size_t total_block_pickups(void) const = 0;

  /**
   * \brief Should return the # of blocks a given cache has had dropped in it
   * this timestep.
   *
   * This is currently has a max of 1, due to limitations/shortcuts taken with
   * the block drop events.
   */
  virtual size_t total_block_drops(void) const = 0;
};

NS_END(caches, metrics, arena, cosm);

