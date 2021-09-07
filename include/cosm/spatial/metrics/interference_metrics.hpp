/**
 * \file interference_metrics.hpp
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

#ifndef INCLUDE_COSM_SPATIAL_METRICS_INTERFERENCE_METRICS_HPP_
#define INCLUDE_COSM_SPATIAL_METRICS_INTERFERENCE_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class interference_metrics
 * \ingroup spatial metrics
 *
 * \brief Interface defining what metrics should be collected regarding
 * inter-robot interference as robots go about their tasks in the arena.
 */
class interference_metrics : public virtual rmetrics::base_metrics {
 public:
  interference_metrics(void) = default;
  ~interference_metrics(void) override = default;

  /**
   * \brief If \c TRUE, then a robot is currently experiencing inter-robot
   * interference.
   */
  virtual bool exp_interference(void) const = 0;

  /**
   * \brief If \c TRUE, then a robot has just begun experiencing inter-robot
   * interference. This should return \c FALSE on all subsequent steps the robot
   * is experiencing interference.
   */
  virtual bool entered_interference(void) const = 0;

  /**
   * \brief If \c TRUE, then a robot has just exited interference avoidance. This
   * should return \c FALSE on all previous steps the robot is in interference
   * avoidance, and all steps afterwards when it returns to normal operation.
   */
  virtual bool exited_interference(void) const = 0;

  /**
   * \brief If \ref exited_interference() returns \c TRUE, then this should
   * return the duration of the interference in timesteps.
   */
  virtual rtypes::timestep interference_duration(void) const = 0;

  /**
   * \brief When \ref facing_interference() returns \c TRUE, then this should
   * return the robot's current discrete position in 3D. If the robot is only
   * moving in 2D, then the Z component should always be 0.
   */
  virtual rmath::vector3z interference_loc3D(void) const = 0;
};

NS_END(metrics, spatial, cosm);

#endif /* INCLUDE_COSM_SPATIAL_METRICS_INTERFERENCE_METRICS_HPP_ */
