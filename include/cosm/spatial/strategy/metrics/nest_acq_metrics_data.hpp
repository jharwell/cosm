/**
 * \file nest_acq_metrics_data.hpp
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

#ifndef INCLUDE_COSM_SPATIAL_STRATEGY_METRICS_NEST_ACQ_METRICS_DATA_HPP_
#define INCLUDE_COSM_SPATIAL_STRATEGY_METRICS_NEST_ACQ_METRICS_DATA_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <atomic>

#include "rcppsw/metrics/base_data.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, strategy, metrics, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief Container for holding collected statistics. Must be atomic so counts
 * are valid in parallel metric collection contexts.
 */
struct nest_acq_metrics_data {
  std::atomic<double> random_thresh{0.0};
  std::atomic_size_t  n_random_thresh{0};
};

NS_END(detail);

struct nest_acq_metrics_data : public rmetrics::base_data {
  detail::nest_acq_metrics_data interval{};
  detail::nest_acq_metrics_data cum{};
};

NS_END(metrics, strategy, spatial, cosm);

#endif /* INCLUDE_COSM_SPATIAL_STRATEGY_METRICS_NEST_ACQ_METRICS_DATA_HPP_ */
