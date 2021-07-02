/**
 * \file block_cluster_metrics_data.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_FORAGING_METRICS_BLOCK_CLUSTER_METRICS_DATA_HPP_
#define INCLUDE_COSM_FORAGING_METRICS_BLOCK_CLUSTER_METRICS_DATA_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <atomic>
#include <vector>

#include "rcppsw/metrics/base_metrics_data.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, foraging, metrics, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
struct cluster_extent {
  std::atomic<double> area;
  std::atomic<double> xmin;
  std::atomic<double> xmax;
  std::atomic<double> ymin;
  std::atomic<double> ymax;
};
/**
 * \struct block_cluster_metrics_data
 * \ingroup foraging block_dist metrics detail
 *
 * \brief Container for holding collected statistics of \ref
 * block_cluster_metrics.
 */
struct block_cluster_metrics_data  {
  explicit block_cluster_metrics_data(size_t n_clusters)
      : block_counts(n_clusters) {}

  std::vector<std::atomic_size_t> block_counts;
};

NS_END(detail);

struct block_cluster_metrics_data : public rmetrics::base_metrics_data {
  explicit block_cluster_metrics_data(size_t n_clusters)
      : interval{n_clusters},
        cum{n_clusters},
        extents{n_clusters} {}

  detail::block_cluster_metrics_data  interval;
  detail::block_cluster_metrics_data  cum;
  std::vector<detail::cluster_extent> extents{};
};

NS_END(metrics, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_METRICS_BLOCK_CLUSTER_METRICS_DATA_HPP_ */
