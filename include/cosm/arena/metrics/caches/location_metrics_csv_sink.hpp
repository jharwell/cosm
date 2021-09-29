/**
 * \file location_metrics_csv_sink.hpp
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

#ifndef INCLUDE_COSM_ARENA_METRICS_CACHES_LOCATION_METRICS_CSV_SINK_HPP_
#define INCLUDE_COSM_ARENA_METRICS_CACHES_LOCATION_METRICS_CSV_SINK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/ds/metrics/grid2D_metrics_csv_sink.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, arena, metrics, caches);
class location_metrics_collector;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class location_metrics_csv_sink final :
    public rdmetrics::grid2D_metrics_csv_sink<rdmetrics::cell_avg> {
 public:
  using collector_type = location_metrics_collector;
  using rdmetrics::grid2D_metrics_csv_sink<rdmetrics::cell_avg>::grid2D_metrics_csv_sink;
};

NS_END(metrics, caches, arena, cosm);

#endif /* INCLUDE_COSM_ARENA_METRICS_CACHES_LOCATION_METRICS_CSV_SINK_HPP_ */
