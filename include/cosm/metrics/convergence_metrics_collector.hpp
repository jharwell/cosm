/**
 * @file convergence_metrics_collector.hpp
 *
 * @copyright 2018 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_METRICS_CONVERGENCE_METRICS_COLLECTOR_HPP_
#define INCLUDE_COSM_METRICS_CONVERGENCE_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <list>

#include "rcppsw/metrics/base_metrics_collector.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * @class convergence_metrics_collector
 * @ingroup cosm metrics
 *
 * @brief Collector for \ref convergence_metrics.
 *
 * Metrics are written out each timestep.
 */
class convergence_metrics_collector final : public rmetrics::base_metrics_collector {
 public:
  /**
   * @param ofname The output file name.
   * @param interval Collection interval.
   */
  convergence_metrics_collector(const std::string& ofname,
                                uint interval);

  void reset(void) override;
  void collect(const rmetrics::base_metrics& metrics) override;

 private:
  struct convergence_measure_stats {
    double raw{0.0};
    double norm{0.0};
    uint   converged{0};
  };

  std::list<std::string> csv_header_cols(void) const override;
  bool csv_line_build(std::string& line) override;
  void reset_after_interval(void) override;

  /* clang-format off */
  double                           m_conv_epsilon{0.0};
  struct convergence_measure_stats m_interact_stats{};
  struct convergence_measure_stats m_order_stats{};
  struct convergence_measure_stats m_pos_ent_stats{};
  struct convergence_measure_stats m_tdist_ent_stats{};
  /* clang-format on */
};

NS_END(metrics, cosm);

#endif /* INCLUDE_COSM_METRICS_CONVERGENCE_METRICS_COLLECTOR_HPP_ */
