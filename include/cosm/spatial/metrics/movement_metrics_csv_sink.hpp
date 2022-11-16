/**
 * \file movement_metrics_csv_sink.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <vector>
#include <list>

#include "rcppsw/metrics/csv_sink.hpp"
#include "rcppsw/spatial/euclidean_dist.hpp"

#include "cosm/cosm.hpp"
#include "cosm/spatial/metrics/movement_category.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, metrics);
class movement_metrics_collector;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class movement_metrics_csv_sink
 * \ingroup spatial metrics
 *
 * \brief Sink for \ref movement_metrics and \ref movement_metrics_collector to
 * output metrics to .csv.
 */
class movement_metrics_csv_sink final : public rmetrics::csv_sink {
 public:
  using collector_type = movement_metrics_collector;

  /**
   * \brief \see rmetrics::csv_sink.
   */
  movement_metrics_csv_sink(fs::path fpath_no_ext,
                            const rmetrics::output_mode& mode,
                            const rtypes::timestep& interval);

  /* csv_sink overrides */
  std::list<std::string> csv_header_cols(
      const rmetrics::base_data* data) const override;

  boost::optional<std::string> csv_line_build(
      const rmetrics::base_data* data,
      const rtypes::timestep& t) override;
};

NS_END(metrics, spatial, cosm);

