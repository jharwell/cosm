/**
 * \file nest_zone_metrics.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_SPATIAL_METRICS_NEST_ZONE_METRICS_HPP_
#define INCLUDE_COSM_SPATIAL_METRICS_NEST_ZONE_METRICS_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/types/spatial_dist.hpp"
#include "rcppsw/metrics/base_metrics.hpp"
#include "rcppsw/math/vector3.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class nest_zone_metrics
 * \ingroup spatial metrics
 *
 * \brief Metrics to be collected about robots as the enter/exit/move in the
 * nest.
 */
class nest_zone_metrics : public virtual rmetrics::base_metrics {
 public:
  nest_zone_metrics(void) = default;
  ~nest_zone_metrics(void) override = default;

  /* Not move/copy constructable/assignable by default */
  nest_zone_metrics(const nest_zone_metrics&) = delete;
  nest_zone_metrics& operator=(const nest_zone_metrics&) = delete;
  nest_zone_metrics(nest_zone_metrics&&) = delete;
  nest_zone_metrics& operator=(nest_zone_metrics&&) = delete;

  /**
   * \brief If \c TRUE, then a robot is currently in the nest.
   */
  virtual bool in_nest(void) const = 0;

  /**
   * \brief If \c TRUE, then a robot has just begun entered the nest. This
   * should return \c FALSE on all subsequent steps the robot is in the nest.
   */
  virtual bool entered_nest(void) const = 0;

  /**
   * \brief If \c TRUE, then a robot has just exited the nest. This should
   * return \c FALSE on all previous steps the robot is in the nest, and all
   * steps afterwards.
   */
  virtual bool exited_nest(void) const = 0;

  /**
   * \brief If \ref exited_nest() returns \c TRUE, then this should
   * return the # timesteps the robot was in the nest.
   */
  virtual rtypes::timestep nest_duration(void) const = 0;

  /**
   * \brief If \ref entered_nest() returns \c TRUE, then this should
   * return the current timestep.
   */
  virtual rtypes::timestep nest_entry_time(void) const = 0;
};

NS_END(metrics, spatial, cosm);

#endif /* INCLUDE_COSM_SPATIAL_METRICS_NEST_ZONE_METRICS_HPP_ */
