/**
 * \file location_metrics.hpp
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
#include "rcppsw/math/vector2.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::arena::metrics::caches {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class location_metrics
 * \ingroup arena metrics caches
 *
 * \brief Defines the metrics to be collected from a \ref carepr::arena_cache
 * regarding its location in the arena.
 *
 * Metrics are collected every timestep.
 */
class location_metrics : public virtual rmetrics::base_metrics {
 public:
  location_metrics(void) = default;

  /**
   * \brief Should return the discrete location of the cache.
   */
  virtual rcppsw::math::vector2z location(void) const = 0;
};

} /* namespace cosm::arena::metrics::caches */
