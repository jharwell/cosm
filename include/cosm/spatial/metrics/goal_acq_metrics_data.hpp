/**
 * \file goal_acq_metrics_data.hpp
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

#ifndef INCLUDE_COSM_SPATIAL_METRICS_GOAL_ACQ_METRICS_DATA_HPP_
#define INCLUDE_COSM_SPATIAL_METRICS_GOAL_ACQ_METRICS_DATA_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <atomic>

#include "rcppsw/metrics/base_data.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, spatial, metrics, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \brief Container for holding collected statistics. Must be atomic so counts
 * are valid in parallel metric collection contexts.
 */
struct goal_acq_data {
  std::atomic_size_t n_true_exploring_for_goal{0};
  std::atomic_size_t n_false_exploring_for_goal{0};
  std::atomic_size_t n_vectoring_to_goal{0};
  std::atomic_size_t n_acquiring_goal{0};
};

NS_END(detail);

struct goal_acq_metrics_data : public rmetrics::base_data {
  detail::goal_acq_data interval{};
  detail::goal_acq_data cum{};
};

NS_END(metrics, spatial, cosm);

#endif /* INCLUDE_COSM_SPATIAL_METRICS_GOAL_ACQ_METRICS_DATA_HPP_ */
