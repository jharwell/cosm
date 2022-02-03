/**
 * \file bi_tab_metrics_data.hpp
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

#ifndef INCLUDE_COSM_TA_METRICS_BI_TAB_METRICS_DATA_HPP_
#define INCLUDE_COSM_TA_METRICS_BI_TAB_METRICS_DATA_HPP_

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
struct bi_tab_metrics_data {
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


NS_END(detail);

struct bi_tab_metrics_data : public rmetrics::base_data {
  detail::bi_tab_metrics_data interval{};
  detail::bi_tab_metrics_data cum{};
};

NS_END(metrics, ta, cosm);

#endif /* INCLUDE_COSM_TA_METRICS_BI_TAB_METRICS_DATA_HPP_ */
