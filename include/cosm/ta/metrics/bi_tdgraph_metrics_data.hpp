/**
 * \file bi_tdgraph_metrics_data.hpp
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

