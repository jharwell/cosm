/**
 * \file ucb1_allocator.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/ucb1_allocator.hpp"

#include <algorithm>

#include "cosm/ta/polled_task.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
polled_task* ucb1_allocator::operator()(const std::vector<polled_task*>& tasks,
                                        size_t alloc_count) const {
  ER_INFO("n_tasks=%zu, n_allocs=%zu", tasks.size(), alloc_count);

  auto min_cost = [&](const auto* t1, const auto* t2) {
    ta::time_estimate cost1 =
        t1->task_exec_estimate() -
        std::sqrt(2 * std::log(alloc_count) / t1->task_exec_count());
    ta::time_estimate cost2 =
        t2->task_exec_estimate() -
        std::sqrt(2 * std::log(alloc_count) / t2->task_exec_count());
    return cost1 < cost2;
  };

  auto min_task = std::min_element(tasks.begin(), tasks.end(), min_cost);

  /* Only tasks that have equivalent minimum cost are eligible for selection */
  auto is_equiv_min = [&](const auto* e) {
    return e->task_exec_estimate() == (*min_task)->task_exec_estimate();
  };

  std::vector<polled_task*> equiv_min_tasks;
  std::copy_if(tasks.begin(),
               tasks.end(),
               std::back_inserter(equiv_min_tasks),
               is_equiv_min);

  ER_ASSERT(equiv_min_tasks.size() >= 1, "No minimum cost task found?");

  /*
   * If there is more than one task with the same cost estimate, any of them
   * are OK to allocate, so pick randomly.
   */
  return equiv_min_tasks[m_rng->uniform(
      rmath::rangez(0, equiv_min_tasks.size() - 1))];
} /* alloc_ucb1() */

} /* namespace cosm::ta */
