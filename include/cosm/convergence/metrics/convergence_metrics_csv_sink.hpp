/**
 * \file convergence_metrics_csv_sink.hpp
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

#ifndef INCLUDE_COSM_CONVERGENCE_METRICS_CONVERGENCE_METRICS_CSV_SINK_HPP_
#define INCLUDE_COSM_CONVERGENCE_METRICS_CONVERGENCE_METRICS_CSV_SINK_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <string>

#include "rcppsw/metrics/csv_sink.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, convergence, metrics);
class convergence_metrics_collector;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class convergence_metrics_csv_sink
 * \ingroup convergence metrics
 *
 * \brief Sink for \ref convergence_metrics and \ref
 * convergence_metrics_collector to output metrics to .csv.
 */
class convergence_metrics_csv_sink final : public rmetrics::csv_sink {
 public:
  using collector_type = convergence_metrics_collector;

  /**
   * \brief \see rmetrics::csv_sink.
   */
  convergence_metrics_csv_sink(fs::path fpath_no_ext,
                               const rmetrics::output_mode& mode,
                               const rtypes::timestep& interval);

  /* csv_sink overrides */
  std::list<std::string> csv_header_cols(
      const rmetrics::base_metrics_data* data) const override;

  boost::optional<std::string> csv_line_build(
      const rmetrics::base_metrics_data* data,
      const rtypes::timestep& t) override;
};

NS_END(convergence, metrics, cosm);

#endif /* INCLUDE_COSM_CONVERGENCE_METRICS_CONVERGENCE_METRICS_CSV_SINK_HPP_ */