/**
 * \file interference_locs3D_pos_metrics_csv_sink.hpp
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

#ifndef INCLUDE_COSM_SPATIAL_METRICS_INTERFERENCE_LOCS3D_METRICS_CSV_SINK_HPP_
#define INCLUDE_COSM_SPATIAL_METRICS_INTERFERENCE_LOCS3D_METRICS_CSV_SINK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/spatial/grid3D_metrics_csv_sink.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, metrics);
class interference_locs3D_metrics_collector;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class interference_locs3D_metrics_csv_sink final :
    public rmspatial::grid3D_metrics_csv_sink<rmetrics::spatial::cell_avg> {
 public:
  using collector_type = interference_locs3D_metrics_collector;
  using rmspatial::grid3D_metrics_csv_sink<rmetrics::spatial::cell_avg>::grid3D_metrics_csv_sink;
};

NS_END(metrics, spatial, cosm);

#endif /* INCLUDE_COSM_SPATIAL_METRICS_INTERFERENCE_LOCS3D_METRICS_CSV_SINK_HPP_ */
