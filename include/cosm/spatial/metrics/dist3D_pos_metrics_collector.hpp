/**
 * \file dist3D_pos_metrics_collector.hpp
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

#ifndef INCLUDE_COSM_SPATIAL_METRICS_DIST3D_POS_METRICS_COLLECTOR_HPP_
#define INCLUDE_COSM_SPATIAL_METRICS_DIST3D_POS_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <string>

#include "rcppsw/metrics/spatial/grid3D_metrics_collector.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class dist3D_pos_metrics_collector
 * \ingroup rcppsw metrics swarm spatial
 *
 * \brief Collector for \ref dist3D_metrics.
 *
 * Metrics MUST be collected serially; concurrent updates to the gathered stats
 * are not supported.
 */
class dist3D_pos_metrics_collector final
    : public rmetrics::spatial::grid3D_metrics_collector<rmetrics::spatial::cell_avg> {
 public:
  /**
   * \param ofname The output file name.
   * \param interval Collection interval.
   * \param mode The selected output mode.
   * \param dims Dimensions of arena.
   */
  dist3D_pos_metrics_collector(const std::string& ofname,
                               const rtypes::timestep& interval,
                               const rmetrics::output_mode& mode,
                               const rmath::vector3z& dims)
      : grid3D_metrics_collector(ofname, interval, mode, dims) {}

  void collect(const rmetrics::base_metrics& metrics) override;
};

NS_END(metrics, spatial, cosm);

#endif /* INCLUDE_COSM_SPATIAL_METRICS_DIST3D_POS_METRICS_COLLECTOR_HPP_ */
