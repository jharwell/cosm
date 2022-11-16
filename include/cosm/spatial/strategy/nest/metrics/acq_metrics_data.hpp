/**
 * \file acq_metrics_data.hpp
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
#include "rcppsw/al/multithread.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy, nest, metrics, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief Container for holding collected statistics. Must be atomic so counts
 * are valid in parallel metric collection contexts.
 */
struct acq_metrics_data {
  ral::mt_double_t random_thresh{0.0};
  ral::mt_size_t  n_random_thresh{0};
};

NS_END(detail);

struct acq_metrics_data : public rmetrics::base_data {
  detail::acq_metrics_data interval{};
  detail::acq_metrics_data cum{};
};

NS_END(metrics, nest, strategy, spatial, cosm);
