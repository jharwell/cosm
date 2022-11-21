/**
 * \file epsilon_greedy_allocator.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/epsilon_greedy_allocator.hpp"

#include <cmath>

#include "cosm/ta/strict_greedy_allocator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta {

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
polled_task*
epsilon_greedy_allocator::operator()(const std::vector<polled_task*>& tasks,
                                     size_t alloc_count) const {
  double epsilon = 0;

  if (kRegretBoundLinear == mc_config->regret_bound) {
    epsilon = mc_config->epsilon;
    ER_INFO("Epsilon greedy: n_tasks=%zu, epsilon=%f", tasks.size(), epsilon);
  } else if (kRegretBoundLog == mc_config->regret_bound) {
    double term1 = 1.0;
    double term2 =
        (kC * tasks.size()) / (std::pow(mc_config->epsilon, 2) * alloc_count);
    epsilon = std::min(term1, term2);
    ER_INFO("Epsilon N-greedy: n_tasks=%zu,alloc_count=%zu,epsilon=%f",
            tasks.size(),
            alloc_count,
            epsilon);

  } else {
    ER_FATAL_SENTINEL("Bad epsilon greedy regret bound: %s",
                      mc_config->regret_bound.c_str())
  }

  /*
   * Choose the greedy best task with probability 1.0 - epsilon. If there are
   * multiple best tasks, then a random one will be picked which will not
   * affect our regret bound.
   */
  if (m_rng->bernoulli(1.0 - epsilon)) {
    return strict_greedy_allocator(m_rng)(tasks);
  }
  /* otherwise, pick randomly */
  return tasks[m_rng->uniform(rmath::rangez(0, tasks.size() - 1))];
} /* operator()() */

} /* namespace cosm::ta */
