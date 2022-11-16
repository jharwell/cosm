/**
 * \file ucb1_allocator.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <vector>

#include "rcppsw/common/common.hpp"
#include "rcppsw/er/client.hpp"
#include "rcppsw/math/rng.hpp"
#include "rcppsw/rcppsw.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, ta);
class polled_task;

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
/**
 * \class ucb1_allocator
 * \ingroup ta
 *
 * \brief Allocates a task from a given set of a tasks, using the UCB1 approach
 * from \cite Pini2012,\cite Auer2002, treating the task allocation problem as a
 * multi-armed bandit and achieving a logarithmic regret bound.
 */
class ucb1_allocator : public rer::client<ucb1_allocator> {
 public:
  explicit ucb1_allocator(rmath::rng* rng)
      : ER_CLIENT_INIT("cosm.ta.ucb1_allocator"), m_rng(rng) {}

  /* Not copy constructable/assignable by default */
  ucb1_allocator(const ucb1_allocator&) = delete;
  const ucb1_allocator& operator=(const ucb1_allocator&) = delete;

  /**
   * \brief Perform task allocation.
   *
   * \param tasks The current set of tasks.
   * \param alloc_count The total # of task allocations so far.
   */
  polled_task* operator()(const std::vector<polled_task*>& tasks,
                          size_t alloc_count) const;

 private:
  /* clang-format off */
  rmath::rng* m_rng;
  /* clang-format on */
};

NS_END(ta, cosm);
