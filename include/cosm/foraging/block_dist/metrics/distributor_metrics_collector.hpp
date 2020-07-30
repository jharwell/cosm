/**
 * \file distributor_metrics_collector.hpp
 *
 * \copyright 2020 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_FORAGING_BLOCK_DIST_METRICS_DISTRIBUTOR_METRICS_COLLECTOR_HPP_
#define INCLUDE_COSM_FORAGING_BLOCK_DIST_METRICS_DISTRIBUTOR_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <list>
#include <atomic>

#include "rcppsw/metrics/base_metrics_collector.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, block_dist, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class distributor_metrics_collector
 * \ingroup cosm foraging block_dist metrics
 *
 * \brief Collector for \ref distributor_metrics.
 *
 * Metrics are written out at the specified collection interval.
 */
class distributor_metrics_collector final : public rmetrics::base_metrics_collector {
 public:
  /**
   * \param ofname_stem The output file name stem.
   * \param interval Collection interval.
   */
  distributor_metrics_collector(const std::string& ofname_stem,
                                const rtypes::timestep& interval);

  void reset(void) override;
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;

 private:
  /**
   * \brief Container for holding distributored statistics. Must be atomic so
   * counts are valid in parallel metric collection contexts.
   */
  struct stats {
    /**
     * \brief  Total # blocks distributored in interval.
     */
    std::atomic_size_t n_configured_clusters{0};

    /**
     * \brief  Total # cube blocks distributored in interval.
     */
    std::atomic_size_t n_mapped_clusters{0};

    /**
     * \brief  Total # ramp blocks distributored in interval.
     */
    std::atomic_size_t capacity{0};

    /**
     * \brief Total # distributorers for distributored blocks in interval.
     */
    std::atomic_size_t size{0};
  };

  std::list<std::string> csv_header_cols(void) const override;
  boost::optional<std::string> csv_line_build(void) override;

  /* clang-format off */
  struct stats m_interval{};
  struct stats m_cum{};
  /* clang-format on */
};

NS_END(metrics, block_dist, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_BLOCK_DIST_METRICS_DISTRIBUTOR_METRICS_COLLECTOR_HPP_ */
