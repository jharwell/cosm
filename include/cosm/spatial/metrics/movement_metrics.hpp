/**
 * \file movement_metrics.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_SPATIAL_METRICS_MOVEMENT_METRICS_HPP_
#define INCLUDE_COSM_SPATIAL_METRICS_MOVEMENT_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/types/spatial_dist.hpp"

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
  virtual rtypes::spatial_dist ts_distance(
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

#endif /* INCLUDE_COSM_SPATIAL_METRICS_MOVEMENT_METRICS_HPP_ */
