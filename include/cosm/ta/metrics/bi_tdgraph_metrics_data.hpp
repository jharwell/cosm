/**
 * \file bi_tdgraph_metrics_data.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>
#include <cmath>

#include "rcppsw/metrics/base_data.hpp"
#include "rcppsw/al/multithread.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, metrics, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
struct bi_tdgraph_metrics_data {
  explicit bi_tdgraph_metrics_data(size_t decomposition_depth)
      : depth_counts(decomposition_depth + 1),
        task_counts(
          static_cast<size_t>(std::pow(2, decomposition_depth + 1) - 1)),
      tab_counts(
          static_cast<size_t>(std::pow(2, decomposition_depth - 1) + 1))
  {}

  std::vector<ral::mt_size_t> depth_counts;
  std::vector<ral::mt_size_t> task_counts;
  std::vector<ral::mt_size_t> tab_counts;
};

NS_END(detail);

struct bi_tdgraph_metrics_data : public rmetrics::base_data {
  explicit bi_tdgraph_metrics_data(size_t decomposition_depth)
      : interval(decomposition_depth),
        cum(decomposition_depth) {}

  detail::bi_tdgraph_metrics_data interval;
  detail::bi_tdgraph_metrics_data cum;
};

NS_END(metrics, ta, cosm);

