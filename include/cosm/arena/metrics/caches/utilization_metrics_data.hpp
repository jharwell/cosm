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
namespace cosm::arena::metrics::caches {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief All stats are cumulative within an interval.
 */
struct utilization_metrics_data_impl {
  size_t n_blocks{0};
  size_t n_pickups{0};
  size_t n_drops{0};
  size_t cache_count{0};
};

struct utilization_metrics_data : public rmetrics::base_data {
  utilization_metrics_data_impl interval{};
  utilization_metrics_data_impl cum{};
};

} /* namespace cosm::arena::metrics::caches */
