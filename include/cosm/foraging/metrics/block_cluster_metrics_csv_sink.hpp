/**
 * \file block_cluster_metrics_csv_sink.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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
namespace cosm::foraging::metrics {

class block_cluster_metrics_collector;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_cluster_metrics_csv_sink
 * \ingroup foraging metrics
 *
 * \brief Sink for \ref block_cluster_metrics and \ref
 * block_cluster_metrics_collector to output metrics to .csv.
 */
class block_cluster_metrics_csv_sink final : public rmetrics::csv_sink {
 public:
  using collector_type = block_cluster_metrics_collector;

  /**
   * \brief \see rmetrics::csv_sink.
   */
  block_cluster_metrics_csv_sink(fs::path fpath_no_ext,
                                 const rmetrics::output_mode& mode,
                                 const rtypes::timestep& interval);

  /* csv_sink overrides */
  std::list<std::string> csv_header_cols(
      const rmetrics::base_data* data) const override;

  boost::optional<std::string> csv_line_build(
      const rmetrics::base_data* data,
      const rtypes::timestep& t) override;
};

} /* namespace cosm::foraging::metrics */

