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

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/metrics/base_data.hpp"
#include "rcppsw/al/multithread.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, foraging, metrics, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
struct cluster_extent {
  ral::mt_double_t area;
  ral::mt_double_t xmin;
  ral::mt_double_t xmax;
  ral::mt_double_t ymin;
  ral::mt_double_t ymax;
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

  std::vector<ral::mt_size_t> block_counts;
};

NS_END(detail);

struct block_cluster_metrics_data : public rmetrics::base_data {
  explicit block_cluster_metrics_data(size_t n_clusters)
      : interval{n_clusters},
        cum{n_clusters},
        extents{n_clusters} {}

  detail::block_cluster_metrics_data  interval;
  detail::block_cluster_metrics_data  cum;
  std::vector<detail::cluster_extent> extents{};

  /**
   * \brief Accumulate data. We ignore the "cum" field on \p rhs, and accumulate
   * into our "cum" field using the "interval" field of \p rhs.
   *
   * This is the most meaningful semantics I could come up with; I couldn't find
   * a way to justify accumulating already cumulative data again (it would have
   * required some additional changes/contortions elsewhere).
   */
  block_cluster_metrics_data& operator+=(const block_cluster_metrics_data& rhs) {
    for (size_t i = 0; i < this->interval.block_counts.size(); ++i) {
      ral::mt_accum(this->interval.block_counts[i],
                    rhs.interval.block_counts[i]);
      ral::mt_accum(this->cum.block_counts[i],
                    rhs.interval.block_counts[i]);
    } /* for(i..) */

    /* no need to accum extents; will always be the same */
    return *this;
  }
};

NS_END(metrics, foraging, cosm);
