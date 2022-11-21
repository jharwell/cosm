/**
 * \file bi_tdgraph_allocator.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>

#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/rcppsw.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::ta {
class polled_task;
namespace ds {
class bi_tdgraph;
} /* namespace ds */
namespace config {
struct task_alloc_config;
} /* namespace config */

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class bi_tdgraph_allocator
 * \ingroup ta
 *
 * \brief Allocate a task from a \ref cta::ds::bi_tdgraph according to a
 * specified policy.
 */
class bi_tdgraph_allocator : public rer::client<bi_tdgraph_allocator> {
 public:
  /**
   * \brief Allocate a task by choosing a random vertex within the graph and
   * return it.
   */
  static inline const std::string kPolicyRandom = "random";

  /**
   * \brief Allocate a task using the eplison-greedy method from Pini2012,
   * Auer2002.
   */
  static inline const std::string kPolicyEplisonGreedy = "epsilon_greedy";

  /**
   * \brief Allocate a task using a matroid optimization approach (strict
   * greedy). No stochasticity is applied.
   */
  static inline const std::string kPolicyStrictGreedy = "strict_greedy";

  /**
   * \brief Allocate a task using a stochastic neighborhood approach.
   *
   * Take the task with the minimum execution time within a neighborhood,
   * applying stochasticity to the task allocation process to attempt to make it
   * more robust/flexible. A neighborhood is defined as all tasks reachable from
   * the most recently executed task within some distance. For this method, that
   * distance is set to 1.
   */
  static inline const std::string kPolicyStochNBHD1 = "stoch_nbhd1";

  /**
   * \brief Allocate a task using the UCB1 policy from Auer2002,Pini2012.
   */
  static inline const std::string kPolicyUCB1 = "UCB1";

  bi_tdgraph_allocator(const config::task_alloc_config* config,
                       ds::bi_tdgraph* graph,
                       rmath::rng* rng)
      : ER_CLIENT_INIT("cosm.ta.bi_tdgraph_allocator"),
        mc_config(config),
        m_graph(graph),
        m_rng(rng) {}

  bi_tdgraph_allocator(const bi_tdgraph_allocator&) = delete;
  bi_tdgraph_allocator& operator=(const bi_tdgraph_allocator&) = delete;

  /**
   * \brief Allocate a task from a bi_tdgraph according to configuration
   *
   * \param current_task The most recently executed task (just finished).
   * \param alloc_count The total # of task allocations so far.
   */
  polled_task* operator()(const polled_task* current_task,
                          size_t alloc_count) const;

  /* clang-format off */
  const config::task_alloc_config* mc_config;

  ds::bi_tdgraph*                  m_graph;
  rmath::rng*                      m_rng;
  /* clang-format on */
};

} /* namespace cosm::ta */
