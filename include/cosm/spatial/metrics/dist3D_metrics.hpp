/**
 * \file dist3D_metrics.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/rcppsw.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::spatial::metrics {

/******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dist3D_metrics
 * \ingroup spatial metrics
 *
 * \brief Defines the metrics to be collected from swarms regarding spatial
 * distributions of robots within 3D space.
 */
class dist3D_metrics : public virtual rmetrics::base_metrics {
 public:
  dist3D_metrics(void) = default;

  /**
   * \brief Return a single robot's current position in 3D space in real
   * coordinates.
   */
  virtual rmath::vector3d rpos3D(void) const = 0;

  /**
   * \brief Return a single robot's discretized position in 3D space.
   */
  virtual rmath::vector3z dpos3D(void) const = 0;

  /**
   * \brief Return a single robot's azimuth angle in 3D space.
   */
  virtual rmath::radians azimuth(void) const = 0;

  /**
   * \brief Return a single robot's zenith angle in 3D space.
   */
  virtual rmath::radians zenith(void) const = 0;
};

} /* namespace cosm::spatial::metrics */
