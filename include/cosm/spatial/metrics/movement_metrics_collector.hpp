/**
 * \file movement_metrics_collector.hpp
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

#ifndef INCLUDE_COSM_SPATIAL_METRICS_MOVEMENT_METRICS_COLLECTOR_HPP_
#define INCLUDE_COSM_SPATIAL_METRICS_MOVEMENT_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <atomic>
#include <vector>
#include <list>

#include "rcppsw/metrics/base_metrics_collector.hpp"
#include "rcppsw/types/spatial_dist.hpp"

#include "cosm/cosm.hpp"
#include "cosm/spatial/metrics/movement_category.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, spatial, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class movement_metrics_collector
 * \ingroup spatial metrics
 *
 * \brief Collector for \ref movement_metrics.
 *
 * Metrics CAN be collected in parallel from robots; concurrent updates to the
 * gathered stats are supported. Metrics are written out at the end of the
 * specified interval.
 */
class movement_metrics_collector final : public rmetrics::base_metrics_collector {
 public:
  /**
   * \param ofname_stem The output file name stem.
   * \param interval Collection interval.
   */
  movement_metrics_collector(const std::string& ofname_stem,
                             const rtypes::timestep& interval);

  void reset(void) override;
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;

 private:
  /**
   * \brief Container for holding collected statistics. Must be atomic so counts
   * are valid in parallel metric collection contexts. Ideally the distances
   * would be atomic \ref rtypes::spatial_dist, but that type does not meet the
   * std::atomic requirements.
   */
  struct stats {
    std::atomic<double> distance{0.0};
    std::atomic_size_t  n_robots{0};
    std::atomic<double> velocity{0.0};
  };

  std::list<std::string> csv_header_cols(void) const override;
  boost::optional<std::string>csv_line_build(void) override;

  /* clang-format off */
  std::vector<stats> m_interval{rcppsw::as_underlying(movement_category::ekMAX)};
  std::vector<stats> m_cum{rcppsw::as_underlying(movement_category::ekMAX)};
  /* clang-format on */
};

NS_END(metrics, spatial, cosm);

#endif /* INCLUDE_COSM_SPATIAL_METRICS_MOVEMENT_METRICS_COLLECTOR_HPP_ */
