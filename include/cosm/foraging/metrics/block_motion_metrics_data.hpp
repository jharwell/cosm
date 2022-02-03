/**
 * \file block_motion_metrics_data.hpp
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

#ifndef INCLUDE_COSM_FORAGING_METRICS_BLOCK_MOTION_METRICS_DATA_HPP_
#define INCLUDE_COSM_FORAGING_METRICS_BLOCK_MOTION_METRICS_DATA_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_data.hpp"
#include "rcppsw/al/multithread.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, foraging, metrics, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \struct block_motion_metrics_data_impl
 * \ingroup foraging block_dist metrics detail
 *
 * \brief Container for holding collected statistics of \ref
 * block_motion_metrics.
 */
struct block_motion_metrics_data_impl  {
  /**
   * \brief  Total # blocks moved.
   */
  ral::mt_size_t n_moved{0};
};

NS_END(detail);

struct block_motion_metrics_data : public rmetrics::base_data {
  detail::block_motion_metrics_data_impl interval{};
  detail::block_motion_metrics_data_impl cum{};
};

NS_END(metrics, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_METRICS_BLOCK_MOTION_METRICS_DATA_HPP_ */
