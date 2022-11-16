/**
 * \file acq_metrics_csv_sink.hpp
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
#include <list>

#include "rcppsw/metrics/csv_sink.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, strategy, nest, metrics);
class acq_metrics_collector;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class acq_metrics_csv_sink
 * \ingroup spatial strategy nest metrics
 *
 * \brief Sink for \ref acq_metrics and \ref acq_metrics_collector to
 * output metrics to .csv.
 */
class acq_metrics_csv_sink final : public rmetrics::csv_sink {
 public:
  using collector_type = acq_metrics_collector;

  /**
   * \brief \see rmetrics::csv_sink.
   */
  acq_metrics_csv_sink(fs::path fpath_no_ext,
                       const rmetrics::output_mode& mode,
                       const rtypes::timestep& interval);

  /* csv_sink overrides */
  std::list<std::string> csv_header_cols(
      const rmetrics::base_data* data) const override;

  boost::optional<std::string> csv_line_build(
      const rmetrics::base_data* data,
      const rtypes::timestep& t) override;
};

NS_END(metrics, nest, strategy, spatial, cosm);
