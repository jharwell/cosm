/**
 * \file bi_tdgraph_allocator.cpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/ta/bi_tdgraph_allocator.hpp"

#include <algorithm>
#include <vector>

#include "cosm/ta/ds/bi_tdgraph.hpp"
#include "cosm/ta/epsilon_greedy_allocator.hpp"
#include "cosm/ta/executable_task.hpp"
#include "cosm/ta/polled_task.hpp"
#include "cosm/ta/random_allocator.hpp"
#include "cosm/ta/stoch_nbhd1_allocator.hpp"
#include "cosm/ta/strict_greedy_allocator.hpp"
#include "cosm/ta/ucb1_allocator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta);

/*******************************************************************************
 * Member Functions
 ******************************************************************************/
polled_task* bi_tdgraph_allocator::operator()(const polled_task* current_task,
                                              size_t alloc_count) const {
  std::vector<polled_task*> tasks(m_graph->n_vertices());
  m_graph->walk(
      [&](polled_task* task) { tasks[m_graph->vertex_id(task)] = task; });

  if (kPolicyRandom == mc_config->policy) {
    return random_allocator(m_rng)(tasks);
  } else if (kPolicyEplisonGreedy == mc_config->policy) {
    return epsilon_greedy_allocator(&mc_config->epsilon_greedy,
                                    m_rng)(tasks, alloc_count);
  } else if (kPolicyStrictGreedy == mc_config->policy) {
    return strict_greedy_allocator(m_rng)(tasks);
  } else if (kPolicyStochNBHD1 == mc_config->policy) {
    return stoch_nbhd1_allocator(m_rng, m_graph)(current_task);
  } else if (kPolicyUCB1 == mc_config->policy) {
    return ucb1_allocator(m_rng)(tasks, alloc_count);
  }
  ER_FATAL_SENTINEL("Bad allocation policy '%s'", mc_config->policy.c_str());
  return nullptr;
}

NS_END(ta, cosm);
