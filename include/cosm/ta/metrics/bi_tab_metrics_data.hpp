/**
 * \file bi_tab_metrics_data.hpp
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
struct bi_tab_metrics_data_impl {
  /**
   * \brief # Times subtask 1 was chosen if partitioning was employed.
   */
  ral::mt_size_t   subtask1_count{0};

  /**
   * \brief # Times subtask 2 was chosen if partitioning was employed.
   */
  ral::mt_size_t   subtask2_count{0};

  /**
   * \brief # Times partitioning was employed when allocating a task,
   */
  ral::mt_size_t   partition_count{0};

  /**
   * \brief # Times partitioning was not employed when allocating a task.
   */
  ral::mt_size_t   no_partition_count{0};

  /**
   * \brief # Times when task allocation resulted in a different task being
   * executed.
   */
  ral::mt_size_t   task_sw_count{0};

  /**
   * \brief # Times when task allocation resulted in a task of a different
   * depth being executed than previous.
   */
  ral::mt_size_t   task_depth_sw_count{0};


  /**
   * \brief The average partitioning probability of the root task in the TAB.
   */
  ral::mt_double_t partition_prob{0.0};

  /**
   * \brief The average subtask selection probability of the root task in the
   * TAB.
   */
  ral::mt_double_t subtask_sel_prob{0.0};
};


struct bi_tab_metrics_data : public rmetrics::base_data {
  bi_tab_metrics_data_impl interval{};
  bi_tab_metrics_data_impl cum{};
};

} /* namespace cosm::ta::metrics */
