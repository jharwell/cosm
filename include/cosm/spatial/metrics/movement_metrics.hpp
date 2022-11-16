/**
 * \file movement_metrics.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/spatial/euclidean_dist.hpp"

#include "cosm/cosm.hpp"
#include "cosm/spatial/metrics/movement_category.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class movement_metrics
 * \ingroup spatial metrics
 *
 * \brief Interface defining what metrics regarding movement/distance traveled
 * should be collected from all controllers.
 */
class movement_metrics : virtual public rmetrics::base_metrics {
 public:
  movement_metrics(void) = default;
  ~movement_metrics(void) override = default;

  /**
   * \brief Get the distance that a robot has traveled in a single timestep. If
   * the robot's motion does not fall into the specified category this timestep,
   * then the distance traveled should be 0.
   *
   * \param category The category of motion to get the distance traveled for.
   */
  virtual rspatial::euclidean_dist ts_distance(
      const movement_category& category) const = 0;

  /**
   * \brief Get the velocity that a robot has on a single timestep. If the robot
   * is only moving in 2D, then the Z component of the velocity should be 0. If
   * the robot's motion does not fall into the specified category this timestep,
   * then the velocity should be 0.
   *
   * \param category The category of motion to get the currently velocity for.
   */
  virtual rmath::vector3d ts_velocity(
      const movement_category& category) const = 0;
};

NS_END(metrics, spatial, cosm);

