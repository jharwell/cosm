/**
 * \file interference_locs2D_metrics_csv_sink.hpp
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
namespace cosm::spatial::metrics {
class interference_locs2D_metrics_collector;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
class interference_locs2D_metrics_csv_sink final:
    public rdmetrics::grid2D_metrics_csv_sink<rdmetrics::cell_avg> {
 public:
  using collector_type = interference_locs2D_metrics_collector;
  using rdmetrics::grid2D_metrics_csv_sink<rdmetrics::cell_avg>::grid2D_metrics_csv_sink;
};

} /* namespace cosm::spatial::metrics */
