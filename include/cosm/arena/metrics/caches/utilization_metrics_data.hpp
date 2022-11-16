/**
 * \file utilization_metrics_data.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

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

