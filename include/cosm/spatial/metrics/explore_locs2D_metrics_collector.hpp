/**
 * \file explore_locs2D_metrics_collector.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_SPATIAL_METRICS_EXPLORE_LOCS2D_METRICS_COLLECTOR_HPP_
#define INCLUDE_COSM_SPATIAL_METRICS_EXPLORE_LOCS2D_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/metrics/spatial/grid2D_metrics_collector.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class explore_locs2D_metrics_collector
 * \ingroup spatial metrics
 *
 * \brief Collector for robot exploration trajectories for goal acquisition,
 * which is collected as a 2D array, and needs its own collector separate from
 * the \ref goal_acq_metrics_collector (1 .csv per collector).
 *
 * Metrics CAN be collected concurrently if the calling context guarantees that
 * no two robots will have the same discrete location. Otherwise, serial
 * collection is required.
 */
class explore_locs2D_metrics_collector final : public rmetrics::spatial::grid2D_metrics_collector<rmetrics::spatial::cell_avg> {
 public:
  /**
   * \param ofname The output file name.
   * \param interval Collection interval.
   * \param dims Dimensions of the arena.
   * \param mode The selected output mode.
   */
  explore_locs2D_metrics_collector(const std::string& ofname,
                                         const rtypes::timestep& interval,
                                         const rmetrics::output_mode& mode,
                                         const rmath::vector2z& dims) :
      grid2D_metrics_collector(ofname, interval, mode, dims) {}

  void collect(const rmetrics::base_metrics& metrics) override;
};

NS_END(metrics, spatial, cosm);

#endif /* INCLUDE_COSM_SPATIAL_METRICS_EXPLORE_LOCS2D_METRICS_COLLECTOR_HPP_ */
