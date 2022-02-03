/**
 * \file distributor_metrics_data.hpp
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

#ifndef INCLUDE_COSM_FORAGING_BLOCK_DIST_METRICS_DISTRIBUTOR_METRICS_DATA_HPP_
#define INCLUDE_COSM_FORAGING_BLOCK_DIST_METRICS_DISTRIBUTOR_METRICS_DATA_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_data.hpp"


/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, foraging, block_dist, metrics, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct distributor_metrics_data
 * \ingroup foraging block_dist metrics detail
 *
 * \brief Container for holding collected statistics of \ref
 * distributor_metrics.
 */
struct distributor_metrics_data  {
  /**
   * \brief  Total # blocks distributed in interval.
   */
  size_t n_configured_clusters{0};

  /**
   * \brief  Total # cube blocks distributed in interval.
   */
  size_t n_mapped_clusters{0};

  /**
   * \brief  Total # ramp blocks distributed in interval.
   */
  size_t capacity{0};

  /**
   * \brief Total # distributorers for distributed blocks in interval.
   */
  size_t size{0};
};

NS_END(detail);

struct distributor_metrics_data : public rmetrics::base_data {
  detail::distributor_metrics_data cum{};
  detail::distributor_metrics_data interval{};
};

NS_END(metrics, block_dist, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_BLOCK_DIST_METRICS_DISTRIBUTOR_METRICS_DATA_HPP_ */
