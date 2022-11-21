/**
 * \file kinematics_metrics_avg_csv_sink.hpp
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
#include "cosm/kin/metrics/contexts.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::kin::metrics {

class kinematics_metrics_collector;
class kinematics_metrics_data;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class kinematics_metrics_avg_csv_sink
 * \ingroup kin metrics
 *
 * \brief Sink for \ref kinematics_metrics and \ref kinematics_metrics_collector to
 *        output \a averaged kinematics metrics to .csv.
 */
class kinematics_metrics_avg_csv_sink final : public rmetrics::csv_sink {
 public:
  using collector_type = kinematics_metrics_collector;

  /**
   * \brief \see rmetrics::csv_sink.
   */
  kinematics_metrics_avg_csv_sink(fs::path fpath_no_ext,
                              const rmetrics::output_mode& mode,
                              const rtypes::timestep& interval);

  /* csv_sink overrides */
  std::list<std::string> csv_header_cols(
      const rmetrics::base_data* data) const override;

  boost::optional<std::string> csv_line_build(
      const rmetrics::base_data* data,
      const rtypes::timestep& t) override;

 private:
  std::string build_from_context(const context_type& ctx,
                                 const kinematics_metrics_data* data,
                                 const rtypes::timestep& t);
};

} /* namespace cosm::kin::metrics */
