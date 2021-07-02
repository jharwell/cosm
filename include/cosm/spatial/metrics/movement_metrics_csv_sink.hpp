/**
 * \file movement_metrics_csv_sink.hpp
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

#ifndef INCLUDE_COSM_SPATIAL_METRICS_MOVEMENT_METRICS_CSV_SINK_HPP_
#define INCLUDE_COSM_SPATIAL_METRICS_MOVEMENT_METRICS_CSV_SINK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <atomic>
#include <vector>
#include <list>

#include "rcppsw/metrics/csv_sink.hpp"
#include "rcppsw/types/spatial_dist.hpp"

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
      const rmetrics::base_metrics_data* data) const override;

  boost::optional<std::string> csv_line_build(
      const rmetrics::base_metrics_data* data,
      const rtypes::timestep& t) override;
};

NS_END(metrics, spatial, cosm);

#endif /* INCLUDE_COSM_SPATIAL_METRICS_MOVEMENT_METRICS_CSV_SINK_HPP_ */