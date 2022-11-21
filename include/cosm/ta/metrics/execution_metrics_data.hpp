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
namespace cosm::ta::metrics {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
struct execution_metrics_data_impl {
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

struct execution_metrics_data : public rmetrics::base_data {
  execution_metrics_data_impl interval{};
  execution_metrics_data_impl cum{};
};

} /* namespace cosm::ta::metrics */

