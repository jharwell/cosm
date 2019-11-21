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

#ifndef INCLUDE_COSM_FSM_METRICS_MOVEMENT_METRICS_HPP_
#define INCLUDE_COSM_FSM_METRICS_MOVEMENT_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/math/vector2.hpp"
#include "rcppsw/types/spatial_dist.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, fsm, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class movement_metrics
 * \ingroup fsm metrics
 *
 * \brief Interface defining what metrics regarding movement traveled should be
 * collected from all controller.
 */
class movement_metrics : virtual public rmetrics::base_metrics {
 public:
  movement_metrics(void) = default;
  ~movement_metrics(void) override = default;

  /**
   * \brief Get the movement that a robot has traveled in a single timestep.
   *
   * This will be called every timestep by the \ref movement_metrics_collector
   * on all controller.
   */
  virtual rtypes::spatial_dist distance(void) const = 0;

  /**
   * \brief Get the velocity that a robot has on a single timestep.
   */
  virtual rmath::vector2d velocity(void) const = 0;
};

NS_END(metrics, fsm, cosm);

#endif /* INCLUDE_COSM_FSM_METRICS_MOVEMENT_METRICS_HPP_ */
