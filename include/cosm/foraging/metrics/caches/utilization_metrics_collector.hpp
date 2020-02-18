/**
 * \file utilization_metrics_collector.hpp
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

#ifndef INCLUDE_COSM_FORAGING_METRICS_CACHES_UTILIZATION_METRICS_COLLECTOR_HPP_
#define INCLUDE_COSM_FORAGING_METRICS_CACHES_UTILIZATION_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <list>
#include <string>

#include "rcppsw/metrics/base_metrics_collector.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, metrics, caches);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class utilization_metrics_collector
 * \ingroup foraging metrics caches
 *
 * \brief Collector for \ref utilization_metrics.
 *
 * Metrics MUST be collected serially; concurrent updates to the gathered stats
 * are not supported. Metrics are output at the specified interval.
 */
class utilization_metrics_collector final : public rmetrics::base_metrics_collector {
 public:
  /**
   * \param ofname Output file name.
   * \param interval Collection interval.
   */
  utilization_metrics_collector(const std::string& ofname,
                                const rtypes::timestep& interval);

  void reset(void) override;
  void reset_after_interval(void) override;
  void collect(const rmetrics::base_metrics& metrics) override;

 private:
  /**
   * \brief All stats are cumulative within an interval.
   */
  struct stats {
    uint n_blocks{0};
    uint n_pickups{0};
    uint n_drops{0};
    uint cache_count{0};
  };

  std::list<std::string> csv_header_cols(void) const override;
  boost::optional<std::string> csv_line_build(void) override;

  /* clang-format off */
  struct stats m_interval{};
  struct stats m_cum{};
  /* clang-format on */
};

NS_END(caches, metrics, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_METRICS_CACHES_UTILIZATION_METRICS_COLLECTOR_HPP_ */
