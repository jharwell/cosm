/**
 * \file execution_metrics_data.hpp
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
NS_START(cosm, ta, metrics, detail);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
struct execution_metrics_data {
  /**
   * \brief # Times the task has been completed.
   */
  ral::mt_size_t complete_count{0};

  /**
   * \brief # Times the task has been aborted.
   */
  ral::mt_size_t abort_count{0};

  /**
   * \brief # Times at their interface.
   */
  ral::mt_size_t interface_count{0};

  /**
   * \brief Execution times of the task.
   */
  ral::mt_size_t exec_time{};

  /**
   * \brief Interface time of the task.
   */
  ral::mt_size_t interface_time{0};

  ral::mt_size_t exec_estimate{0};
  ral::mt_size_t interface_estimate{0};
};

NS_END(detail);

struct execution_metrics_data : public rmetrics::base_data {
  detail::execution_metrics_data interval{};
  detail::execution_metrics_data cum{};
};

NS_END(metrics, ta, cosm);

