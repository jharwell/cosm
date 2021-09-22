/**
 * \file goal_acq_locs2D_metrics_collector.hpp
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

#ifndef INCLUDE_COSM_SPATIAL_METRICS_GOAL_ACQ_LOCS2D_METRICS_COLLECTOR_HPP_
#define INCLUDE_COSM_SPATIAL_METRICS_GOAL_ACQ_LOCS2D_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <memory>

#include "rcppsw/ds/metrics/grid2D_metrics_collector.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class goal_acq_locs2D_metrics_collector
 * \ingroup spatial metrics
 *
 * \brief Collector for \ref goal_acq_metrics goal locations, which is
 * collected as a 2D array, and needs its own collector separate from the \ref
 * goal_acq_metrics_collector (1 .csv per collector).
 *
 * Metrics CAN be collected concurrently if the calling context guarantees that
 * no two robots will have the same discrete location. Otherwise, serial
 * collection is required.
 */
class goal_acq_locs2D_metrics_collector final : public rdmetrics::grid2D_metrics_collector {
 public:
  /**
   * \param sink The metrics sink to use.
   * \param dims Dimensions of arena.
   */
  goal_acq_locs2D_metrics_collector(
      std::unique_ptr<rmetrics::base_metrics_sink> sink,
      const rmath::vector2z& dims);

  void collect(const rmetrics::base_metrics& metrics) override;

};

NS_END(metrics, spatial, cosm);

#endif /* INCLUDE_COSM_SPATIAL_METRICS_GOAL_ACQ_LOCS2D_METRICS_COLLECTOR_HPP_ */
