/**
 * \file explore_locs2D_pos_metrics_csv_sink.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/ds/metrics/grid2D_metrics_csv_sink.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, metrics);
class explore_locs2D_metrics_collector;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class explore_locs2D_metrics_csv_sink final :
    public rdmetrics::grid2D_metrics_csv_sink<rdmetrics::cell_avg> {
 public:
  using collector_type = explore_locs2D_metrics_collector;
  using rdmetrics::grid2D_metrics_csv_sink<rdmetrics::cell_avg>::grid2D_metrics_csv_sink;
};

NS_END(metrics, spatial, cosm);

