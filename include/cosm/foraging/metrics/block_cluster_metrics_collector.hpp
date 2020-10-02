/**
 * \file block_cluster_metrics_collector.hpp
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

#ifndef INCLUDE_COSM_FORAGING_METRICS_BLOCK_CLUSTER_METRICS_COLLECTOR_HPP_
#define INCLUDE_COSM_FORAGING_METRICS_BLOCK_CLUSTER_METRICS_COLLECTOR_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include <list>
#include <atomic>
#include <vector>

#include "rcppsw/metrics/base_metrics_collector.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, metrics);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class block_cluster_metrics_collector
 * \ingroup cosm foraging metrics
 *
 * \brief Collector for \ref block_cluster_metrics.
 *
 * Metrics are written out at the specified collection interval.
 */
class block_cluster_metrics_collector final : public rmetrics::base_metrics_collector {
 public:
  /**
   * \param ofname_stem The output file name stem.
   * \param interval Collection interval.
   * \param n_clusters How many clusters are in the arena?
   */
  block_cluster_metrics_collector(const std::string& ofname_stem,
                                  const rtypes::timestep& interval,
                                  size_t n_clusters);

  void reset(void) override;
  void collect(const rmetrics::base_metrics& metrics) override;
  void reset_after_interval(void) override;

 private:
  struct cluster_extent {
    std::atomic<double> area;
    std::atomic<double> xmin;
    std::atomic<double> xmax;
    std::atomic<double> ymin;
    std::atomic<double> ymax;
  };

  std::list<std::string> csv_header_cols(void) const override;
  boost::optional<std::string> csv_line_build(void) override;

  /* clang-format off */
  std::vector<std::atomic_size_t> m_int_block_counts{};
  std::vector<std::atomic_size_t> m_cum_block_counts{};

  std::vector<cluster_extent>     m_extents{};
  /* clang-format on */
};

NS_END(metrics, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_METRICS_BLOCK_CLUSTER_METRICS_COLLECTOR_HPP_ */
