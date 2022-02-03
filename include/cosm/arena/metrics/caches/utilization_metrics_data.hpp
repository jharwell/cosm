/**
 * \file utilization_metrics_data.hpp
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

#ifndef INCLUDE_COSM_ARENA_METRICS_CACHES_UTILIZATION_METRICS_DATA_HPP_
#define INCLUDE_COSM_ARENA_METRICS_CACHES_UTILIZATION_METRICS_DATA_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/metrics/base_data.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, arena, metrics, caches, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief All stats are cumulative within an interval.
 */
struct utilization_metrics_data {
  size_t n_blocks{0};
  size_t n_pickups{0};
  size_t n_drops{0};
  size_t cache_count{0};
};

NS_END(detail);

struct utilization_metrics_data : public rmetrics::base_data {
  detail::utilization_metrics_data interval{};
  detail::utilization_metrics_data cum{};
};

NS_END(caches, metrics, arena, cosm);

#endif /* INCLUDE_COSM_ARENA_METRICS_CACHES_UTILIZATION_METRICS_DATA_HPP_ */
