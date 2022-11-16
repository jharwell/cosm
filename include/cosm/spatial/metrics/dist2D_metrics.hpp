/**
 * \file dist2D_metrics.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/rcppsw.hpp"
#include "rcppsw/math/radians.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, metrics);

/******************************************************************************
 * Class Definitions
 ******************************************************************************/

/**
 * \class dist2D_metrics
 * \ingroup spatial metrics
 *
 * \brief Defines the metrics to be collected from swarms regarding spatial
 * distributions of robots within 2D space.
 */
class dist2D_metrics : public virtual rmetrics::base_metrics {
 public:
  dist2D_metrics(void) = default;

  /**
   * \brief Return a single robot's current position in 2D space in real
   * coordinates.
   */
  virtual rmath::vector2d rpos2D(void) const = 0;

  /**
   * \brief Return a single robot's discretized position in 2D space.
   */
  virtual rmath::vector2z dpos2D(void) const = 0;

  /**
   * \brief Return a single robot's current heading in 2D space.
   */
  virtual rmath::radians heading2D(void) const = 0;
};

NS_END(metrics, spatial, cosm);

