/**
 * \file execution_metrics_data.hpp
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

#ifndef INCLUDE_COSM_TA_METRICS_EXECUTION_METRICS_DATA_HPP_
#define INCLUDE_COSM_TA_METRICS_EXECUTION_METRICS_DATA_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <atomic>

#include "rcppsw/metrics/base_metrics_data.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta, metrics, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
struct execution_metrics_data {
  /**
   * \brief # Times the task has been completed.
   */
  std::atomic_size_t complete_count{0};

  /**
   * \brief # Times the task has been aborted.
   */
  std::atomic_size_t abort_count{0};

  /**
   * \brief # Times at their interface.
   */
  std::atomic_size_t interface_count{0};

  /**
   * \brief Execution times of the task.
   */
  std::atomic_size_t exec_time{};

  /**
   * \brief Interface time of the task.
   */
  std::atomic_size_t interface_time{0};

  std::atomic_size_t exec_estimate{0};
  std::atomic_size_t interface_estimate{0};
};

NS_END(detail);

struct execution_metrics_data : public rmetrics::base_metrics_data {
  detail::execution_metrics_data interval{};
  detail::execution_metrics_data cum{};
};

NS_END(metrics, ta, cosm);

#endif /* INCLUDE_COSM_TA_METRICS_EXECUTION_METRICS_DATA_HPP_ */
